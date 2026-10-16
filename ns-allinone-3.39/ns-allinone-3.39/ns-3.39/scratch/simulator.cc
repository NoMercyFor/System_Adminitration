/* smart-warehouse-csma-ca.cc  –  ns-3.39
- CSMA/CA (without RTS/CTS), Wi-Fi 802.11n Ad-hoc, 30x30m Warehouse
- Build: cp *.cc <ns3>/scratch/ && ./ns3 run "scratch/smart-warehouse-csma-ca --nNodes=10"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

int main(int argc, char *argv[])
{
    uint32_t nNodes  = 10;
    double   simTime = 20.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("nNodes",  "Total number of nodes (1 gateway + N robots)", nNodes);
    cmd.AddValue("simTime", "Simulation time (s)",                          simTime);
    cmd.Parse(argc, argv);

    NS_ASSERT_MSG(nNodes >= 2, "At least 2 nodes are required");

    RngSeedManager::SetSeed(42);
    RngSeedManager::SetRun(1);

    uint32_t nRobots = nNodes - 1;

    // ── Nodes ─────────────────────────────────────────────────────────────────
    NodeContainer allNodes; allNodes.Create(nNodes);
    NodeContainer gateway;  gateway.Add(allNodes.Get(0));
    NodeContainer robots;
    for (uint32_t i = 1; i < nNodes; ++i) robots.Add(allNodes.Get(i));

    // ── Wi-Fi 802.11n Ad-hoc ──────────────────────────────────────────────────
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode",        StringValue("HtMcs7"),
        "ControlMode",     StringValue("HtMcs0"),
        "RtsCtsThreshold", UintegerValue(999999));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    // Simple channel, no complex loss models added
    YansWifiChannelHelper chHelper;
    chHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chHelper.AddPropagationLoss("ns3::FriisPropagationLossModel",
        "Frequency", DoubleValue(2.4e9));

    YansWifiPhyHelper phy;
    phy.SetChannel(chHelper.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd",   DoubleValue(20.0));

    NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);

    // ── Internet Stack ────────────────────────────────────────────────────────
    InternetStackHelper inet; inet.Install(allNodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifaces = ipv4.Assign(devices);

    // ── Mobility Model ────────────────────────────────────────────────────────
    MobilityHelper mob;

    // Fixed Gateway position at (15, 15)
    Ptr<ListPositionAllocator> gwPos = CreateObject<ListPositionAllocator>();
    gwPos->Add(Vector(15.0, 15.0, 0.0));
    mob.SetPositionAllocator(gwPos);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(gateway);

    // Robots: Random initial position within 30x30m area, Random Waypoint model
    Ptr<RandomRectanglePositionAllocator> rpa =
        CreateObject<RandomRectanglePositionAllocator>();
    rpa->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));
    rpa->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));

    mob.SetPositionAllocator(rpa);
    mob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed",             StringValue("ns3::UniformRandomVariable[Min=0.5|Max=1.5]"),
        "Pause",             StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
        "PositionAllocator", PointerValue(rpa));
    mob.Install(robots);

    // ── Applications: UDP CBR 1.5 Mbps, each robot -> Gateway ────────────────
    uint16_t port = 9;

    // Sink on Gateway - listens on ANY address
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sinkHelper.Install(gateway);
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    // Source on each robot
    for (uint32_t i = 0; i < nRobots; ++i)
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
            InetSocketAddress(ifaces.GetAddress(0), port));
        onoff.SetConstantRate(DataRate("1500000bps"), 1400);
        onoff.SetAttribute("OnTime",
            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime",
            StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer app = onoff.Install(robots.Get(i));
        app.Start(Seconds(2.0 + i * 0.1)); // Clearer stagger, starts after 2s
        app.Stop(Seconds(simTime));
    }

    // ── FlowMonitor ───────────────────────────────────────────────────────────
    FlowMonitorHelper fmh;
    Ptr<FlowMonitor> fm = fmh.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ── Results Collection ────────────────────────────────────────────────────
    fm->CheckForLostPackets();
    auto classifier = DynamicCast<Ipv4FlowClassifier>(fmh.GetClassifier());
    auto stats      = fm->GetFlowStats();

    double rxBytes = 0, txPkts = 0, rxPkts = 0, delayNs = 0;

    for (auto &kv : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        // Only measure flows destined for gateway (port 9)
        if (t.destinationPort != port) continue;
        rxBytes += kv.second.rxBytes;
        txPkts  += kv.second.txPackets;
        rxPkts  += kv.second.rxPackets;
        if (kv.second.rxPackets > 0)
            delayNs += kv.second.delaySum.GetNanoSeconds();
    }

    double active     = simTime - 2.0;   // Exclude 2s warm-up period
    double throughput = (rxBytes * 8.0) / (active * 1e6);
    double pdr        = txPkts > 0 ? (rxPkts / txPkts) * 100.0 : 0.0;
    double delayMs    = rxPkts > 0 ? (delayNs / rxPkts) / 1e6  : 0.0;
    double collision  = (100.0 - pdr > 0) ? 100.0 - pdr : 0.0;

    std::cout << "\n===== Smart Warehouse CSMA/CA (No RTS/CTS) =====\n"
              << "  nNodes          : " << nNodes     << "\n"
              << "  Tx Packets      : " << txPkts     << "\n"
              << "  Rx Packets      : " << rxPkts     << "\n"
              << "  Throughput      : " << throughput << " Mbps\n"
              << "  PDR             : " << pdr        << " %\n"
              << "  E2E Delay       : " << delayMs    << " ms\n"
              << "  Collision Proxy : " << collision  << " %\n"
              << "================================================\n\n";

    Simulator::Destroy();
    return 0;
}