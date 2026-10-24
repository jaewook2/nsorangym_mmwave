/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * [nsoran-asm] ASM verification:
 *   1 LTE eNB at origin, mmWave gNBs along x (default 2: left/right).
 *   nMmWaveEnb=3 places a third gNB at the LTE (x,y); mmWaveLteZOffset
 *   raises its z so the two radios are not at the same point.
 *   Default UE layout: cluster around each gNB (left ~ASM 2 load, right ~ASM 4).
 *   ueClusterSidesOnly clusters only on the leftmost/rightmost gNBs.
 *   LTE eNB is 6 RB / 1.4 MHz so it cannot carry the cluster load; mmWave stays 20 MHz.
 *   configuration=2 (28 GHz) isolates the two clusters so OFF dumps to LTE, not the other gNB.
 *   Optional --uePlacement=random keeps the old rectangle + high/mid/low mix.
 *
 * Runtime ASM timeline (absolute time in ms). CSV:
 *   timestampMs,cellId,asmMode
 * cellId 0 means this gNB. Example:
 *   0,2,1
 *   200,2,3
 *   400,2,6
 * ASM 6 TurnOff: HO barred, evict to another mmWave or LTE if none.
 *   ./ns3 run "scratch/scenario-asm-verify --controlFileNameAsm=asm_actions_for_ns3.csv --simTime=1.0"
 *
 * LTE vs mmWave PDCP throughput is written to asm-path-tput.csv every --tputPeriod.
 * Per-UE sat rows also include servingNrCellId and useMmWave for the Gym state.
 *
 * Gym (nsorangym): useSemaphores=1, scheduleAsmControl=false, flushAsmControlFile=false
 * (LTE flushes the shared CSV after all gNBs apply).
 */

#include "ns3/applications-module.h"
#include "ns3/cc-helper.h"
#include "ns3/component-carrier.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-enb-net-device.h"
#include "ns3/lte-enb-rrc.h"
#include "ns3/lte-helper.h"
#include "ns3/mc-enb-pdcp.h"
#include "ns3/mc-ue-net-device.h"
#include "ns3/mmwave-component-carrier-enb.h"
#include "ns3/mmwave-enb-mac.h"
#include "ns3/mmwave-enb-net-device.h"
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-mac-scheduler.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/packet-sink.h"
#include "ns3/point-to-point-helper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <vector>

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("ScenarioAsmVerify");

struct AsmSlotStats
{
  uint32_t nSlots = 0;
  uint32_t nSlotsWithDlCtrl = 0;
  uint32_t nSlotsWithUlCtrl = 0;
  uint64_t sumDataSym = 0;
  uint32_t maxDataSym = 0;
  uint32_t minDataSym = UINT32_MAX;
  uint32_t nSlotsOverBudget = 0;
  uint32_t expectedDataSym = 0;
  uint8_t asmMode = 1;
};

static AsmSlotStats g_asmStats;
static std::ofstream g_slotLog;

static std::string
FormatImsi (uint64_t imsi)
{
  std::ostringstream oss;
  oss << std::setfill ('0') << std::setw (5) << imsi;
  return oss.str ();
}

static Ptr<MmWaveMacScheduler> g_sched;
static std::vector<Ptr<MmWaveEnbNetDevice>> g_gnbs;
static Ptr<LteEnbNetDevice> g_lteEnbDev;
static NetDeviceContainer g_mcUeDevs;
static std::vector<uint8_t> g_ueClass;     // 0=high, 1=mid, 2=low
static std::vector<std::string> g_ueRate;
static std::vector<uint64_t> g_lastUeRx;
static std::ofstream g_ueSatLog;
static uint64_t g_ltePdcpBytes = 0;
static uint64_t g_mmwPdcpBytes = 0;
static std::set<const void *> g_pdcpConnected;
static std::ofstream g_tputLog;

void
CountLtePdcpTx (uint16_t /*rnti*/, uint8_t /*lcid*/, uint32_t size)
{
  g_ltePdcpBytes += size;
}

void
CountMmwPdcpTx (uint16_t /*rnti*/, uint8_t /*lcid*/, uint32_t size)
{
  g_mmwPdcpBytes += size;
}

void
LookupUeDataPath (uint32_t ueIndex, uint16_t *nrCell, bool *useMmw)
{
  *nrCell = 0;
  *useMmw = false;
  if (!g_lteEnbDev || ueIndex >= g_mcUeDevs.GetN ())
    {
      return;
    }
  Ptr<McUeNetDevice> ueDev = DynamicCast<McUeNetDevice> (g_mcUeDevs.Get (ueIndex));
  if (!ueDev || !g_lteEnbDev->GetRrc ())
    {
      return;
    }
  uint64_t imsi = ueDev->GetImsi ();
  uint16_t rnti = g_lteEnbDev->GetRrc ()->GetRntiFromImsi (imsi);
  if (rnti == 0 || !g_lteEnbDev->GetRrc ()->HasUeManager (rnti))
    {
      return;
    }
  Ptr<UeManager> ueMgr = g_lteEnbDev->GetRrc ()->GetUeManager (rnti);
  if (!ueMgr)
    {
      return;
    }
  *nrCell = ueMgr->GetMmWaveCellId ();
  for (auto const &drb : ueMgr->GetDrbMap ())
    {
      Ptr<McEnbPdcp> pdcp = DynamicCast<McEnbPdcp> (drb.second->m_pdcp);
      if (pdcp)
        {
          *useMmw = pdcp->GetUseMmWaveConnection ();
          return;
        }
    }
}

void
ConnectPdcpPathTraces (Ptr<LteEnbNetDevice> lteEnb)
{
  if (!lteEnb || !lteEnb->GetRrc ())
    {
      return;
    }
  for (auto const &ue : lteEnb->GetRrc ()->GetUeMap ())
    {
      for (auto const &drb : ue.second->GetDrbMap ())
        {
          Ptr<McEnbPdcp> pdcp = DynamicCast<McEnbPdcp> (drb.second->m_pdcp);
          if (!pdcp)
            {
              continue;
            }
          const void *key = PeekPointer (pdcp);
          if (g_pdcpConnected.count (key))
            {
              continue;
            }
          pdcp->TraceConnectWithoutContext ("TxPduToLte", MakeCallback (&CountLtePdcpTx));
          pdcp->TraceConnectWithoutContext ("TxPduToMmWave", MakeCallback (&CountMmwPdcpTx));
          g_pdcpConnected.insert (key);
        }
    }
}

void
ReportPathThroughput (Ptr<LteEnbNetDevice> lteEnb, ApplicationContainer sinks, double periodSec,
                      uint64_t *lastLte, uint64_t *lastMmw, uint64_t *lastApp)
{
  ConnectPdcpPathTraces (lteEnb);
  uint64_t dLte = g_ltePdcpBytes - *lastLte;
  uint64_t dMmw = g_mmwPdcpBytes - *lastMmw;
  *lastLte = g_ltePdcpBytes;
  *lastMmw = g_mmwPdcpBytes;
  uint64_t appTotal = 0;
  for (uint32_t i = 0; i < sinks.GetN (); ++i)
    {
      Ptr<PacketSink> sink = DynamicCast<PacketSink> (sinks.Get (i));
      if (sink)
        {
          appTotal += sink->GetTotalRx ();
        }
    }
  uint64_t dApp = (appTotal >= *lastApp) ? (appTotal - *lastApp) : appTotal;
  *lastApp = appTotal;
  std::ostringstream modeStr;
  std::ostringstream stateStr;
  std::ostringstream dataSymStr;
  std::ostringstream cellPowerStr;
  double asmPowerW = 0.0;
  for (uint32_t i = 0; i < g_gnbs.size (); ++i)
    {
      if (i > 0)
        {
          modeStr << ";";
          stateStr << ";";
          dataSymStr << ";";
        }
      modeStr << g_gnbs[i]->GetCellId () << ":" << unsigned (g_gnbs[i]->GetAsmMode ());
      stateStr << g_gnbs[i]->GetCellId () << ":" << unsigned (g_gnbs[i]->GetAsmPhase ()) << ":"
               << unsigned (g_gnbs[i]->GetAsmTarget ());
      int dataSym = 0;
      auto ccMap = g_gnbs[i]->GetCcMap ();
      if (!ccMap.empty ())
        {
          Ptr<MmWaveComponentCarrierEnb> cc =
              DynamicCast<MmWaveComponentCarrierEnb> (ccMap.begin ()->second);
          if (cc && cc->GetMacScheduler ())
            {
              dataSym = cc->GetMacScheduler ()->GetAvailableDataSymbols ();
            }
        }
      dataSymStr << g_gnbs[i]->GetCellId () << ":" << dataSym;
      double cellW = 0.0;
      if (g_gnbs[i]->LogsKpmToFile ())
        {
          cellW = g_gnbs[i]->GetLastAveragePowerW ();
        }
      else
        {
          cellW = g_gnbs[i]->ConsumeAveragePowerW (periodSec);
        }
      if (i > 0)
        {
          cellPowerStr << ";";
        }
      cellPowerStr << g_gnbs[i]->GetCellId () << ":" << cellW;
      asmPowerW += cellW;
    }
  double lteKbps = dLte * 8.0 / 1e3 / periodSec;
  double mmwKbps = dMmw * 8.0 / 1e3 / periodSec;
  double appKbps = dApp * 8.0 / 1e3 / periodSec;
  uint64_t dPath = dLte + dMmw;
  double lteFrac = dPath > 0 ? static_cast<double> (dLte) / dPath : 0.0;
  if (g_tputLog.is_open ())
    {
      g_tputLog << Simulator::Now ().GetMilliSeconds () << "," << modeStr.str () << "," << dLte
                << "," << dMmw << "," << lteKbps << "," << mmwKbps << "," << lteFrac << "," << dApp
                << "," << appKbps << "," << stateStr.str () << "," << asmPowerW << ","
                << dataSymStr.str () << "," << cellPowerStr.str () << std::endl;
    }
  NS_LOG_UNCOND (Simulator::Now ().GetMilliSeconds ()
                 << " ms ASM " << modeStr.str () << " PDCP LTE " << lteKbps << " kbps mmWave "
                 << mmwKbps << " kbps (lteFrac " << lteFrac << ") app " << appKbps << " kbps");
  if (g_ueSatLog.is_open ())
    {
      double minSat = 1.0;
      uint32_t nSat = 0;
      if (g_lastUeRx.size () < sinks.GetN ())
        {
          g_lastUeRx.resize (sinks.GetN (), 0);
        }
      for (uint32_t i = 0; i < sinks.GetN (); ++i)
        {
          Ptr<PacketSink> sink = DynamicCast<PacketSink> (sinks.Get (i));
          uint64_t rx = sink ? sink->GetTotalRx () : 0;
          uint64_t dUe = (rx >= g_lastUeRx[i]) ? (rx - g_lastUeRx[i]) : rx;
          g_lastUeRx[i] = rx;
          double rxKbps = dUe * 8.0 / 1e3 / periodSec;
          double offeredKbps = 0.0;
          uint8_t cls = (i < g_ueClass.size ()) ? g_ueClass[i] : 0;
          std::string rate = (i < g_ueRate.size ()) ? g_ueRate[i] : "0bps";
          offeredKbps = static_cast<double> (DataRate (rate).GetBitRate ()) / 1e3;
          double sat = offeredKbps > 0.0 ? std::min (1.0, rxKbps / offeredKbps) : 0.0;
          minSat = (nSat == 0) ? sat : std::min (minSat, sat);
          nSat++;
          const char *clsName = (cls == 0) ? "high" : (cls == 1) ? "mid" : "low";
          uint16_t nrCell = 0;
          bool useMmw = false;
          LookupUeDataPath (i, &nrCell, &useMmw);
          g_ueSatLog << Simulator::Now ().GetMilliSeconds () << "," << (i + 1) << "," << clsName
                     << "," << rate << "," << rxKbps << "," << offeredKbps << "," << sat << ","
                     << nrCell << "," << (useMmw ? 1 : 0) << std::endl;
        }
      NS_LOG_UNCOND (Simulator::Now ().GetMilliSeconds ()
                     << " ms minSat " << minSat << " over " << nSat << " UEs");
    }
  Simulator::Schedule (Seconds (periodSec), &ReportPathThroughput, lteEnb, sinks, periodSec,
                       lastLte, lastMmw, lastApp);
}

void
ReportAsmSched (MmWaveEnbMac::MmWaveSchedTraceInfo info)
{
  if (g_sched)
    {
      g_asmStats.asmMode = g_sched->GetAsmMode ();
      g_asmStats.expectedDataSym =
          static_cast<uint32_t> (std::max (0, g_sched->GetAvailableDataSymbols ()));
    }
  int dataSym = 0;
  int ctrlSym = 0;
  bool hasDlCtrl = false;
  bool hasUlCtrl = false;
  for (auto const &tti : info.m_indParam.m_slotAllocInfo.m_ttiAllocInfo)
    {
      if (tti.m_ttiType == TtiAllocInfo::CTRL)
        {
          ctrlSym += tti.m_dci.m_numSym;
          if (tti.m_tddMode == TtiAllocInfo::DL_slotAllocInfo)
            {
              hasDlCtrl = true;
            }
          if (tti.m_tddMode == TtiAllocInfo::UL_slotAllocInfo)
            {
              hasUlCtrl = true;
            }
        }
      else
        {
          dataSym += tti.m_dci.m_numSym;
        }
    }

  g_asmStats.nSlots++;
  if (hasDlCtrl)
    {
      g_asmStats.nSlotsWithDlCtrl++;
    }
  if (hasUlCtrl)
    {
      g_asmStats.nSlotsWithUlCtrl++;
    }
  g_asmStats.sumDataSym += static_cast<uint32_t> (dataSym);
  if (static_cast<uint32_t> (dataSym) > g_asmStats.maxDataSym)
    {
      g_asmStats.maxDataSym = static_cast<uint32_t> (dataSym);
    }
  if (static_cast<uint32_t> (dataSym) < g_asmStats.minDataSym)
    {
      g_asmStats.minDataSym = static_cast<uint32_t> (dataSym);
    }
  bool over = (dataSym > static_cast<int> (g_asmStats.expectedDataSym));
  if (over)
    {
      g_asmStats.nSlotsOverBudget++;
    }

  if (g_slotLog.is_open ())
    {
      SfnSf sfn = info.m_indParam.m_sfnSf;
      g_slotLog << Simulator::Now ().GetMilliSeconds () << "," << +sfn.m_frameNum << ","
                << +sfn.m_sfNum << "," << +sfn.m_slotNum << "," << dataSym << "," << ctrlSym << ","
                << g_asmStats.expectedDataSym << "," << (over ? 1 : 0) << ","
                << (hasDlCtrl ? 1 : 0) << "," << (hasUlCtrl ? 1 : 0) << std::endl;
    }
}

void
ForceMmWaveOnlyData (Ptr<LteEnbNetDevice> lteEnb, NetDeviceContainer mcUeDevs)
{
  if (!lteEnb)
    {
      return;
    }
  uint32_t nSet = 0;
  for (uint32_t i = 0; i < mcUeDevs.GetN (); ++i)
    {
      Ptr<McUeNetDevice> ueDev = DynamicCast<McUeNetDevice> (mcUeDevs.Get (i));
      if (!ueDev)
        {
          continue;
        }
      uint16_t rnti = lteEnb->GetRrc ()->GetRntiFromImsi (ueDev->GetImsi ());
      if (rnti == 0)
        {
          NS_LOG_WARN ("IMSI " << ueDev->GetImsi () << " has no LTE RNTI yet");
          continue;
        }
      lteEnb->SetUeQoS (rnti, 0.0);
      nSet++;
    }
  NS_LOG_INFO ("Forced perPckToLTE=0 for " << nSet << " UEs");
}

void
PrintUeAttachment (std::string filename, NodeContainer ueNodes, NetDeviceContainer mcUeDevs,
                   Ptr<LteEnbNetDevice> lteEnb)
{
  std::ofstream outFile (filename.c_str (), std::ios_base::trunc);
  outFile << "imsi,x,y,servingNrCellId,perPckToLte,appClass,appRate" << std::endl;
  for (uint32_t i = 0; i < mcUeDevs.GetN (); ++i)
    {
      Ptr<McUeNetDevice> ueDev = DynamicCast<McUeNetDevice> (mcUeDevs.Get (i));
      Vector pos = ueNodes.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
      uint64_t imsi = ueDev->GetImsi ();
      double qos = -1.0;
      uint16_t nrCell = 0;
      if (lteEnb)
        {
          uint16_t rnti = lteEnb->GetRrc ()->GetRntiFromImsi (imsi);
          if (rnti != 0)
            {
              qos = lteEnb->GetUeQoS (rnti);
              Ptr<UeManager> ueMgr = lteEnb->GetRrc ()->GetUeManager (rnti);
              if (ueMgr)
                {
                  nrCell = ueMgr->GetMmWaveCellId ();
                }
            }
        }
      outFile << imsi << "," << pos.x << "," << pos.y << "," << nrCell << "," << qos << ","
              << ((i < g_ueClass.size () && g_ueClass[i] == 0)
                      ? "high"
                      : (i < g_ueClass.size () && g_ueClass[i] == 1) ? "mid" : "low")
              << "," << ((i < g_ueRate.size ()) ? g_ueRate[i] : "") << std::endl;
      NS_LOG_INFO ("UE IMSI " << imsi << " at (" << pos.x << "," << pos.y << ") NR cell " << nrCell
                              << " perPckToLTE " << qos);
    }
}

static ns3::GlobalValue g_nUe ("nUe", "Total number of UEs",
                               ns3::UintegerValue (30), ns3::MakeUintegerChecker<uint32_t> (1));
static ns3::GlobalValue g_ues ("ues", "Total number of UEs (NsOranEnv datalake key)",
                               ns3::UintegerValue (30), ns3::MakeUintegerChecker<uint32_t> (1));
static ns3::GlobalValue g_nMmWaveEnb ("nMmWaveEnb", "Number of mmWave gNBs (left/right of LTE)",
                                     ns3::UintegerValue (2),
                                     ns3::MakeUintegerChecker<uint32_t> (1, 8));
static ns3::GlobalValue g_gnbDistance (
    "gnbDistance",
    "Distance (m) from the center LTE eNB to each side mmWave gNB.",
    ns3::DoubleValue (200.0), ns3::MakeDoubleChecker<double> (20.0, 5000.0));
static ns3::GlobalValue g_mmWaveLteZOffset (
    "mmWaveLteZOffset",
    "If a mmWave gNB shares the LTE (x,y), add this many metres to z.",
    ns3::DoubleValue (0.5), ns3::MakeDoubleChecker<double> (0.0, 20.0));
static ns3::GlobalValue g_ueClusterSidesOnly (
    "ueClusterSidesOnly",
    "If true, cluster UEs only around the leftmost and rightmost mmWave gNBs.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_simTime ("simTime", "Simulation time in seconds", ns3::DoubleValue (10.0),
                                   ns3::MakeDoubleChecker<double> (0.2, 100.0));
static ns3::GlobalValue g_indicationPeriodicity (
    "indicationPeriodicity",
    "E2/Gym report period in seconds (also the control handshake period).",
    ns3::DoubleValue (0.1), ns3::MakeDoubleChecker<double> (0.01, 2.0));
static ns3::GlobalValue g_useSemaphores (
    "useSemaphores",
    "If true, pause on Gym semaphores and apply ASM CSV after each handshake.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_controlMode (
    "controlMode",
    "LTE control use case. Gym ASM uses 'asm'.",
    ns3::StringValue (""), ns3::MakeStringChecker ());
static ns3::GlobalValue g_handoverMode (
    "handoverMode",
    "Secondary cell handover: NoAuto, FixedTtt, DynamicTtt, or Threshold.",
    ns3::StringValue ("NoAuto"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_controlFileName (
    "controlFileName",
    "LTE control CSV path (Gym handshake). For ASM this can match controlFileNameAsm.",
    ns3::StringValue (""), ns3::MakeStringChecker ());
static ns3::GlobalValue g_asmMode (
    "asmMode",
    "duty_mode for mmWave gNB: 1=100%, 2=75%, 3=50%, 4=25%, 5=0% data, 6=TurnOff. "
    "With asmStateMachine, 1=Active, 2=ASM1, 3=ASM2, 4=ASM3, 5=ASM4, 6=OFF.",
    ns3::UintegerValue (1), ns3::MakeUintegerChecker<uint8_t> (1, 6));
static ns3::GlobalValue g_asmStateMachine (
    "asmStateMachine",
    "If true, asmMode is an ASM target and ns-3 runs component transitions.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
// Salem et al., IEEE NCA 2019, Table I (IMEC/Debaillie sleep levels). On equals off.
// PA 35.5 us (SM1), RF 0.5 ms (SM2), BB 5 ms (SM3), Control/Minimal 0.5 s (SM4).
// Minimal to OFF is not in that table; it reuses the SM4 0.5 s class.
static ns3::GlobalValue g_asmPaOffTime ("asmPaOffTime", "Seconds to turn the PA off.",
                                        ns3::DoubleValue (35.5e-6), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmRfOffTime ("asmRfOffTime", "Seconds to turn RF off.",
                                        ns3::DoubleValue (0.5e-3), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmBbOffTime ("asmBbOffTime", "Seconds to turn the baseband off.",
                                        ns3::DoubleValue (5e-3), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmControlToMinimalTime (
    "asmControlToMinimalTime", "Seconds to reduce Control to Minimal.", ns3::DoubleValue (0.5),
    ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmMinimalOffTime ("asmMinimalOffTime", "Seconds for Minimal to reach OFF.",
                                             ns3::DoubleValue (0.5), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmOffToMinimalTime ("asmOffToMinimalTime", "Seconds to raise OFF to Minimal.",
                                               ns3::DoubleValue (0.5), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmMinimalToControlTime (
    "asmMinimalToControlTime", "Seconds to raise Minimal to Control.", ns3::DoubleValue (0.5),
    ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmBbOnTime ("asmBbOnTime", "Seconds to turn the baseband on.",
                                       ns3::DoubleValue (5e-3), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmRfOnTime ("asmRfOnTime", "Seconds to turn RF on.", ns3::DoubleValue (0.5e-3),
                                       ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmPaOnTime ("asmPaOnTime", "Seconds to turn the PA on.",
                                       ns3::DoubleValue (35.5e-6), ns3::MakeDoubleChecker<double> ());
// Pico sleep totals (Onireti, PIMRC 2017): SM1 1.51, SM2 0.368, SM3 0.301, SM4 0.238 W.
// Control and Minimal are alternatives, so Control is the SM3 total and Minimal the SM4 total.
// Wake power is twice the operating power of that component. PA wake is twice 1 W TxPower.
static ns3::GlobalValue g_asmRfPower ("asmRfPower", "RF operating power in watts.",
                                      ns3::DoubleValue (1.142), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmBbPower ("asmBbPower", "Baseband operating power in watts.",
                                      ns3::DoubleValue (0.067), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmControlPower ("asmControlPower", "Control operating power in watts.",
                                           ns3::DoubleValue (0.301), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmMinimalPower ("asmMinimalPower", "Minimal operating power in watts.",
                                           ns3::DoubleValue (0.238), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmPaWakePower ("asmPaWakePower", "PA wake power in watts.",
                                          ns3::DoubleValue (2.0), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmRfWakePower ("asmRfWakePower", "RF wake power in watts.",
                                          ns3::DoubleValue (2.284), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmBbWakePower ("asmBbWakePower", "Baseband wake power in watts.",
                                          ns3::DoubleValue (0.134), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmControlWakePower ("asmControlWakePower", "Control wake power in watts.",
                                               ns3::DoubleValue (0.602), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_asmMinimalWakePower ("asmMinimalWakePower", "Minimal wake power in watts.",
                                               ns3::DoubleValue (0.476), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_configuration ("configuration",
                                         "RF configuration 0=850MHz, 1=3.5GHz, 2=28GHz",
                                         ns3::UintegerValue (0),
                                         ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_fullBufferRate ("fullBufferRate",
                                          "Unused when per-class rates are set; kept for CLI compat",
                                          ns3::StringValue ("10Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_rateHigh ("rateHigh", "CBR rate for high-load UEs (10 of 30)",
                                    ns3::StringValue ("10Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_rateMid ("rateMid", "CBR rate for mid-load UEs (10 of 30)",
                                   ns3::StringValue ("2Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_rateLow ("rateLow", "CBR rate for low-load UEs (10 of 30, random layout)",
                                   ns3::StringValue ("500kbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_uePlacement (
    "uePlacement",
    "UE layout: cluster around each mmWave gNB, or random rectangle.",
    ns3::StringValue ("cluster"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_ueClusterRadius (
    "ueClusterRadius",
    "Cluster radius (m) around each mmWave gNB when uePlacement=cluster.",
    ns3::DoubleValue (25.0), ns3::MakeDoubleChecker<double> (1.0, 500.0));
static ns3::GlobalValue g_rateLeft (
    "rateLeft",
    "CBR rate for UEs clustered at the left mmWave gNB (heavier, ~ASM 2).",
    ns3::StringValue ("2Mbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_rateRight (
    "rateRight",
    "CBR rate for UEs clustered at the right mmWave gNB (lighter, ~ASM 4).",
    ns3::StringValue ("600kbps"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (10),
                                      ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM",
                                        ns3::BooleanValue (true), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_enableTraces ("enableTraces", "If true, dump ns-3 PHY/MAC/RLC traces",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_ueAreaY (
    "ueAreaY",
    "Random UE placement half-width in y (m). x spans +/- (gnbDistance+ueAreaXPad).",
    ns3::DoubleValue (80.0), ns3::MakeDoubleChecker<double> (10.0, 2000.0));
static ns3::GlobalValue g_ueAreaXPad ("ueAreaXPad", "Extra x margin beyond gNB positions (m)",
                                      ns3::DoubleValue (50.0),
                                      ns3::MakeDoubleChecker<double> (0.0, 2000.0));
static ns3::GlobalValue g_controlFileNameAsm (
    "controlFileNameAsm",
    "ASM timeline/poll CSV (timestampMs,cellId,asmMode). Empty = initial --asmMode only.",
    ns3::StringValue (""), ns3::MakeStringChecker ());
static ns3::GlobalValue g_scheduleAsmControl (
    "scheduleAsmControl",
    "If true, CSV timestamps are absolute sim time in ms and the file is read once.",
    ns3::BooleanValue (true), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_flushAsmControlFile (
    "flushAsmControlFile",
    "If true and scheduleAsmControl=false, truncate the ASM CSV after each poll.",
    ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_asmControlPeriodicity (
    "asmControlPeriodicity",
    "ASM CSV poll period in seconds when scheduleAsmControl=false.",
    ns3::DoubleValue (0.1), ns3::MakeDoubleChecker<double> (0.001, 10.0));
static ns3::GlobalValue g_tputPeriod (
    "tputPeriod",
    "LTE vs mmWave PDCP throughput sample period in seconds.",
    ns3::DoubleValue (0.05), ns3::MakeDoubleChecker<double> (0.01, 2.0));
static ns3::GlobalValue g_lteBandwidthRbs (
    "lteBandwidthRbs",
    "LTE eNB bandwidth in RBs (6=1.4MHz, 15=3, 25=5, 50=10, 75=15, 100=20). mmWave BW unchanged.",
    ns3::UintegerValue (6), ns3::MakeUintegerChecker<uint16_t> (6, 100));

static uint32_t
ClusterIndexForUe (uint32_t u, uint32_t nUe, uint32_t nGnb)
{
  if (nGnb <= 1)
    {
      return 0;
    }
  uint32_t nEach = nUe / nGnb;
  uint32_t rem = nUe % nGnb;
  uint32_t start = 0;
  for (uint32_t g = 0; g < nGnb; ++g)
    {
      uint32_t nThis = nEach + (g < rem ? 1 : 0);
      if (u < start + nThis)
        {
          return g;
        }
      start += nThis;
    }
  return nGnb - 1;
}

static uint32_t
ClusterGnbIndexForUe (uint32_t u, uint32_t nUe, uint32_t nGnb, bool sidesOnly)
{
  if (sidesOnly && nGnb >= 2)
    {
      return ClusterIndexForUe (u, nUe, 2) == 0 ? 0 : (nGnb - 1);
    }
  return ClusterIndexForUe (u, nUe, nGnb);
}

int
main (int argc, char *argv[])
{
  LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnable ("ScenarioAsmVerify", LOG_LEVEL_INFO);

  CommandLine cmd;
  cmd.Parse (argc, argv);

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;

  GlobalValue::GetValueByName ("ues", uintegerValue);
  uint32_t nUeNodes = uintegerValue.Get ();
  GlobalValue::GetValueByName ("nMmWaveEnb", uintegerValue);
  uint32_t nMmWaveEnbNodes = uintegerValue.Get ();
  GlobalValue::GetValueByName ("gnbDistance", doubleValue);
  double gnbDistance = doubleValue.Get ();
  GlobalValue::GetValueByName ("asmMode", uintegerValue);
  uint8_t asmMode = uintegerValue.Get ();
  GlobalValue::GetValueByName ("asmStateMachine", booleanValue);
  bool asmStateMachine = booleanValue.Get ();
  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("enableTraces", booleanValue);
  bool enableTraces = booleanValue.Get ();
  GlobalValue::GetValueByName ("rateHigh", stringValue);
  std::string rateHigh = stringValue.Get ();
  GlobalValue::GetValueByName ("rateMid", stringValue);
  std::string rateMid = stringValue.Get ();
  GlobalValue::GetValueByName ("rateLow", stringValue);
  std::string rateLow = stringValue.Get ();
  GlobalValue::GetValueByName ("uePlacement", stringValue);
  std::string uePlacement = stringValue.Get ();
  GlobalValue::GetValueByName ("ueClusterRadius", doubleValue);
  double ueClusterRadius = doubleValue.Get ();
  GlobalValue::GetValueByName ("mmWaveLteZOffset", doubleValue);
  double mmWaveLteZOffset = doubleValue.Get ();
  GlobalValue::GetValueByName ("ueClusterSidesOnly", booleanValue);
  bool ueClusterSidesOnly = booleanValue.Get ();
  GlobalValue::GetValueByName ("rateLeft", stringValue);
  std::string rateLeft = stringValue.Get ();
  GlobalValue::GetValueByName ("rateRight", stringValue);
  std::string rateRight = stringValue.Get ();
  GlobalValue::GetValueByName ("ueAreaY", doubleValue);
  double ueAreaY = doubleValue.Get ();
  GlobalValue::GetValueByName ("ueAreaXPad", doubleValue);
  double ueAreaXPad = doubleValue.Get ();
  GlobalValue::GetValueByName ("configuration", uintegerValue);
  uint8_t configuration = uintegerValue.Get ();
  GlobalValue::GetValueByName ("controlFileNameAsm", stringValue);
  std::string controlFileNameAsm = stringValue.Get ();
  GlobalValue::GetValueByName ("controlFileName", stringValue);
  std::string controlFileName = stringValue.Get ();
  GlobalValue::GetValueByName ("controlMode", stringValue);
  std::string controlMode = stringValue.Get ();
  GlobalValue::GetValueByName ("scheduleAsmControl", booleanValue);
  bool scheduleAsmControl = booleanValue.Get ();
  GlobalValue::GetValueByName ("flushAsmControlFile", booleanValue);
  bool flushAsmControlFile = booleanValue.Get ();
  GlobalValue::GetValueByName ("asmControlPeriodicity", doubleValue);
  double asmControlPeriodicity = doubleValue.Get ();
  GlobalValue::GetValueByName ("tputPeriod", doubleValue);
  double tputPeriod = doubleValue.Get ();
  GlobalValue::GetValueByName ("indicationPeriodicity", doubleValue);
  double indicationPeriodicity = doubleValue.Get ();
  GlobalValue::GetValueByName ("useSemaphores", booleanValue);
  bool useSemaphores = booleanValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", stringValue);
  std::string handoverMode = stringValue.Get ();
  if (handoverMode != "NoAuto" && handoverMode != "FixedTtt" && handoverMode != "DynamicTtt" &&
      handoverMode != "Threshold")
    {
      NS_FATAL_ERROR ("handoverMode must be NoAuto, FixedTtt, DynamicTtt, or Threshold");
    }

  if (useSemaphores)
    {
      if (controlFileNameAsm.empty ())
        {
          controlFileNameAsm = "asm_actions_for_ns3.csv";
        }
      if (controlFileName.empty ())
        {
          controlFileName = controlFileNameAsm;
        }
      if (controlMode.empty ())
        {
          controlMode = "asm";
        }
      scheduleAsmControl = false;
      // Several gNBs share one CSV; LTE flushes after GymControlReady.
      flushAsmControlFile = false;
      asmControlPeriodicity = indicationPeriodicity;
      tputPeriod = indicationPeriodicity;
    }

  // LTE must not carry user data: 0% of PDCP packets to LTE.
  Config::SetDefault ("ns3::McEnbPdcp::perPckToLTE", DoubleValue (0.0));
  Config::SetDefault ("ns3::MmWaveMacScheduler::AsmMode", UintegerValue (asmMode));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::AsmMode", UintegerValue (asmMode));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::AsmStateMachine", BooleanValue (asmStateMachine));
  auto setAsmTime = [] (const char *globalName, const char *attributeName) {
      ns3::DoubleValue value;
      ns3::GlobalValue::GetValueByName (globalName, value);
      ns3::Config::SetDefault (attributeName, ns3::TimeValue (ns3::Seconds (value.Get ())));
  };
  auto setAsmPower = [] (const char *globalName, const char *attributeName) {
      ns3::DoubleValue value;
      ns3::GlobalValue::GetValueByName (globalName, value);
      ns3::Config::SetDefault (attributeName, ns3::DoubleValue (value.Get ()));
  };
  setAsmTime ("asmPaOffTime", "ns3::MmWaveEnbNetDevice::AsmPaOffTime");
  setAsmTime ("asmRfOffTime", "ns3::MmWaveEnbNetDevice::AsmRfOffTime");
  setAsmTime ("asmBbOffTime", "ns3::MmWaveEnbNetDevice::AsmBbOffTime");
  setAsmTime ("asmControlToMinimalTime", "ns3::MmWaveEnbNetDevice::AsmControlToMinimalTime");
  setAsmTime ("asmMinimalOffTime", "ns3::MmWaveEnbNetDevice::AsmMinimalOffTime");
  setAsmTime ("asmOffToMinimalTime", "ns3::MmWaveEnbNetDevice::AsmOffToMinimalTime");
  setAsmTime ("asmMinimalToControlTime", "ns3::MmWaveEnbNetDevice::AsmMinimalToControlTime");
  setAsmTime ("asmBbOnTime", "ns3::MmWaveEnbNetDevice::AsmBbOnTime");
  setAsmTime ("asmRfOnTime", "ns3::MmWaveEnbNetDevice::AsmRfOnTime");
  setAsmTime ("asmPaOnTime", "ns3::MmWaveEnbNetDevice::AsmPaOnTime");
  setAsmPower ("asmRfPower", "ns3::MmWaveEnbNetDevice::AsmRfPower");
  setAsmPower ("asmBbPower", "ns3::MmWaveEnbNetDevice::AsmBbPower");
  setAsmPower ("asmControlPower", "ns3::MmWaveEnbNetDevice::AsmControlPower");
  setAsmPower ("asmMinimalPower", "ns3::MmWaveEnbNetDevice::AsmMinimalPower");
  setAsmPower ("asmPaWakePower", "ns3::MmWaveEnbNetDevice::AsmPaWakePower");
  setAsmPower ("asmRfWakePower", "ns3::MmWaveEnbNetDevice::AsmRfWakePower");
  setAsmPower ("asmBbWakePower", "ns3::MmWaveEnbNetDevice::AsmBbWakePower");
  setAsmPower ("asmControlWakePower", "ns3::MmWaveEnbNetDevice::AsmControlWakePower");
  setAsmPower ("asmMinimalWakePower", "ns3::MmWaveEnbNetDevice::AsmMinimalWakePower");
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ControlFileNameAsm", StringValue (controlFileNameAsm));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::ScheduleAsmControl", BooleanValue (scheduleAsmControl));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::FlushAsmControlFile", BooleanValue (flushAsmControlFile));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::AsmControlPeriodicity",
                      DoubleValue (asmControlPeriodicity));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeLte", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::MmWaveHelper::E2ModeNr", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::MmWaveHelper::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::LteEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::E2Periodicity", DoubleValue (indicationPeriodicity));
  Config::SetDefault ("ns3::LteEnbNetDevice::ControlFileName", StringValue (controlFileName));
  Config::SetDefault ("ns3::LteEnbNetDevice::ControlMode", StringValue (controlMode));
  Config::SetDefault ("ns3::LteEnbNetDevice::UseSemaphores", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuUpReport", BooleanValue (true));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableCuCpReport", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuUpReport", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableCuCpReport", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableDuReport", BooleanValue (true));
  Config::SetDefault ("ns3::LteEnbNetDevice::EnableE2FileLogging", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::MmWaveEnbNetDevice::EnableE2FileLogging", BooleanValue (useSemaphores));
  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  Config::SetDefault ("ns3::MmWaveEnbMac::NumberOfRaPreambles", UintegerValue (40));
  Config::SetDefault ("ns3::LteEnbMac::NumberOfRaPreambles", UintegerValue (40));
  Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode", StringValue (handoverMode));
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (-5.0));
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                      TimeValue (MilliSeconds (100)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));

  double bandwidth = 20e6;
  double centerFrequency = 850e6;
  int numAntennasMcUe = 1;
  int numAntennasMmWave = 1;
  switch (configuration)
    {
    case 0:
      centerFrequency = 850e6;
      bandwidth = 20e6;
      break;
    case 1:
      centerFrequency = 3.5e9;
      bandwidth = 20e6;
      break;
    case 2:
      centerFrequency = 28e9;
      bandwidth = 20e6;
      numAntennasMcUe = 16;
      numAntennasMmWave = 64;
      break;
    default:
      NS_FATAL_ERROR ("Unknown configuration");
    }

  Config::SetDefault ("ns3::MmWavePhyMacCommon::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (centerFrequency));

  GlobalValue::GetValueByName ("lteBandwidthRbs", uintegerValue);
  uint16_t lteBandwidthRbs = uintegerValue.Get ();
  switch (lteBandwidthRbs)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
      break;
    default:
      NS_FATAL_ERROR ("lteBandwidthRbs must be 6, 15, 25, 50, 75, or 100");
    }

  NS_LOG_UNCOND ("ASM verify: asmMode=" << unsigned (asmMode) << " nMmWaveEnb=" << nMmWaveEnbNodes
                                        << " nUe=" << nUeNodes << " gnbDistance=" << gnbDistance
                                        << " m placement=" << uePlacement
                                        << " clusterR=" << ueClusterRadius
                                        << " rates=" << rateHigh << "/" << rateMid << "/"
                                        << rateLow << " left/right=" << rateLeft << "/" << rateRight
                                        << " freq=" << centerFrequency / 1e9
                                        << " GHz mmWaveBw=" << bandwidth / 1e6
                                        << " MHz lteRbs=" << lteBandwidthRbs
                                        << " handover=" << handoverMode
                                        << " controlFile=" << controlFileNameAsm
                                        << " scheduleAsm=" << scheduleAsmControl
                                        << " useSemaphores=" << useSemaphores);

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  {
    Ptr<CcHelper> lteCcHelper = CreateObject<CcHelper> ();
    lteCcHelper->SetNumberOfComponentCarriers (1);
    lteCcHelper->SetUlEarfcn (18100);
    lteCcHelper->SetDlEarfcn (100);
    lteCcHelper->SetDlBandwidth (lteBandwidthRbs);
    lteCcHelper->SetUlBandwidth (lteBandwidthRbs);
    std::map<uint8_t, ComponentCarrier> lteCcMap = lteCcHelper->EquallySpacedCcs ();
    lteCcMap.at (0).SetAsPrimary (true);
    mmwaveHelper->SetLteCcPhyParams (lteCcMap);
  }
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::ThreeGppUmiStreetCanyonChannelConditionModel");
  mmwaveHelper->SetUePhasedArrayModelAttribute ("NumColumns",
                                               UintegerValue (std::sqrt (numAntennasMcUe)));
  mmwaveHelper->SetUePhasedArrayModelAttribute ("NumRows",
                                               UintegerValue (std::sqrt (numAntennasMcUe)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute ("NumColumns",
                                                UintegerValue (std::sqrt (numAntennasMmWave)));
  mmwaveHelper->SetEnbPhasedArrayModelAttribute ("NumRows",
                                                UintegerValue (std::sqrt (numAntennasMmWave)));

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  ipv4h.Assign (internetDevices);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;
  mmWaveEnbNodes.Create (nMmWaveEnbNodes);
  lteEnbNodes.Create (1);
  ueNodes.Create (nUeNodes);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);

  Vector ltePos (0.0, 0.0, 10.0);
  std::vector<Vector> mmWavePositions;
  if (nMmWaveEnbNodes == 1)
    {
      mmWavePositions.push_back (Vector (0.0, 0.0, 10.0));
    }
  else
    {
      for (uint32_t i = 0; i < nMmWaveEnbNodes; ++i)
        {
          double x = -gnbDistance + (nMmWaveEnbNodes == 1
                                         ? 0.0
                                         : (2.0 * gnbDistance) * i / (nMmWaveEnbNodes - 1));
          mmWavePositions.push_back (Vector (x, 0.0, 10.0));
        }
    }
  for (auto &pos : mmWavePositions)
    {
      double dx = pos.x - ltePos.x;
      double dy = pos.y - ltePos.y;
      if ((dx * dx + dy * dy) < 1e-12)
        {
          pos.z = ltePos.z + mmWaveLteZOffset;
        }
    }

  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (ltePos);
  for (auto const &pos : mmWavePositions)
    {
      enbPositionAlloc->Add (pos);
    }
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);

  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  const bool clusterUes = (uePlacement != "random");
  if (clusterUes)
    {
      Ptr<UniformRandomVariable> rUv = CreateObject<UniformRandomVariable> ();
      Ptr<UniformRandomVariable> thUv = CreateObject<UniformRandomVariable> ();
      Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
      for (uint32_t u = 0; u < nUeNodes; ++u)
        {
          uint32_t gnbIdx =
              ClusterGnbIndexForUe (u, nUeNodes, nMmWaveEnbNodes, ueClusterSidesOnly);
          Vector c = mmWavePositions[gnbIdx];
          double r = ueClusterRadius * std::sqrt (rUv->GetValue (0.0, 1.0));
          double th = thUv->GetValue (0.0, 2.0 * M_PI);
          uePositionAlloc->Add (Vector (c.x + r * std::cos (th), c.y + r * std::sin (th), 1.5));
        }
      uemobility.SetPositionAllocator (uePositionAlloc);
      uemobility.Install (ueNodes);
    }
  else
    {
      Ptr<UniformRandomVariable> xUv = CreateObject<UniformRandomVariable> ();
      xUv->SetAttribute ("Min", DoubleValue (-gnbDistance - ueAreaXPad));
      xUv->SetAttribute ("Max", DoubleValue (gnbDistance + ueAreaXPad));
      Ptr<UniformRandomVariable> yUv = CreateObject<UniformRandomVariable> ();
      yUv->SetAttribute ("Min", DoubleValue (-ueAreaY));
      yUv->SetAttribute ("Max", DoubleValue (ueAreaY));
      Ptr<RandomRectanglePositionAllocator> uePositionAlloc =
          CreateObject<RandomRectanglePositionAllocator> ();
      uePositionAlloc->SetX (xUv);
      uePositionAlloc->SetY (yUv);
      uemobility.SetPositionAllocator (uePositionAlloc);
      uemobility.Install (ueNodes);
      for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
        {
          Ptr<MobilityModel> mob = ueNodes.Get (u)->GetObject<MobilityModel> ();
          Vector p = mob->GetPosition ();
          p.z = 1.5;
          mob->SetPosition (p);
        }
    }

  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting =
          ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (u)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);

  Ptr<LteEnbNetDevice> lteEnb = DynamicCast<LteEnbNetDevice> (lteEnbDevs.Get (0));
  NS_ABORT_MSG_IF (!lteEnb, "LTE eNB cast failed");
  g_lteEnbDev = lteEnb;
  g_mcUeDevs = mcUeDevs;
  g_gnbs.clear ();
  for (uint32_t i = 0; i < mmWaveEnbDevs.GetN (); ++i)
    {
      Ptr<MmWaveEnbNetDevice> gnb = DynamicCast<MmWaveEnbNetDevice> (mmWaveEnbDevs.Get (i));
      NS_ABORT_MSG_IF (!gnb, "gNB cast failed");
      gnb->SetAsmMode (asmMode);
      g_gnbs.push_back (gnb);
      auto ccMap = gnb->GetCcMap ();
      NS_ABORT_MSG_IF (ccMap.empty (), "gNB has no component carrier");
      Ptr<MmWaveComponentCarrierEnb> cc =
          DynamicCast<MmWaveComponentCarrierEnb> (ccMap.begin ()->second);
      NS_ABORT_MSG_IF (!cc || !cc->GetMacScheduler (), "scheduler missing");
      if (i == 0)
        {
          g_sched = cc->GetMacScheduler ();
          g_asmStats.asmMode = g_sched->GetAsmMode ();
          g_asmStats.expectedDataSym =
              static_cast<uint32_t> (std::max (0, g_sched->GetAvailableDataSymbols ()));
        }
      NS_LOG_UNCOND ("mmWave cellId=" << gnb->GetCellId () << " pos=(" << mmWavePositions[i].x
                                      << "," << mmWavePositions[i].y << ","
                                      << mmWavePositions[i].z << ") AsmMode="
                                      << unsigned (gnb->GetAsmMode ()));
    }
  NS_LOG_UNCOND ("LTE cellId=" << lteEnb->GetCellId () << " at (0,0,10) availableDataSymbols="
                               << g_asmStats.expectedDataSym);

  ApplicationContainer sinkApp;
  ApplicationContainer clientApp;
  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                       InetSocketAddress (Ipv4Address::GetAny (), 1234));
  g_ueClass.assign (nUeNodes, 0);
  g_ueRate.assign (nUeNodes, rateHigh);
  std::vector<uint8_t> classPool;
  uint32_t nEach = nUeNodes / 3;
  uint32_t nHigh = nEach + (nUeNodes % 3 > 0 ? 1 : 0);
  uint32_t nMid = nEach + (nUeNodes % 3 > 1 ? 1 : 0);
  uint32_t nLow = nUeNodes - nHigh - nMid;
  classPool.insert (classPool.end (), nHigh, 0);
  classPool.insert (classPool.end (), nMid, 1);
  classPool.insert (classPool.end (), nLow, 2);
  if (!clusterUes)
    {
      Ptr<UniformRandomVariable> shuffleUv = CreateObject<UniformRandomVariable> ();
      for (uint32_t i = classPool.size (); i > 1; --i)
        {
          uint32_t j = shuffleUv->GetInteger (0, i - 1);
          std::swap (classPool[i - 1], classPool[j]);
        }
    }
  std::ofstream classLog ("ue-app-class.txt");
  classLog << "imsi,appClass,appRate,x,y,clusterGnb" << std::endl;
  const char *clsName[] = {"high", "mid", "low"};
  uint32_t nLeft = 0;
  uint32_t nRight = 0;
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      uint32_t gnbIdx =
          ClusterGnbIndexForUe (u, nUeNodes, nMmWaveEnbNodes, ueClusterSidesOnly);
      uint8_t cls;
      std::string rate;
      if (clusterUes)
        {
          bool left = (gnbIdx == 0);
          cls = left ? 0 : 2;
          rate = left ? rateLeft : rateRight;
          if (!ueClusterSidesOnly && nMmWaveEnbNodes > 2 && gnbIdx > 0
              && gnbIdx + 1 < nMmWaveEnbNodes)
            {
              cls = 1;
              rate = rateMid;
            }
          if (left)
            {
              ++nLeft;
            }
          else
            {
              ++nRight;
            }
        }
      else
        {
          cls = classPool[u];
          rate = (cls == 0) ? rateHigh : (cls == 1) ? rateMid : rateLow;
        }
      g_ueClass[u] = cls;
      g_ueRate[u] = rate;
      Vector pos = ueNodes.Get (u)->GetObject<MobilityModel> ()->GetPosition ();
      classLog << (u + 1) << "," << clsName[cls] << "," << rate << "," << pos.x << "," << pos.y
               << "," << (gnbIdx + 2) << std::endl;
      sinkApp.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
      OnOffHelper fullBuffer ("ns3::UdpSocketFactory",
                              InetSocketAddress (ueIpIface.GetAddress (u), 1234));
      fullBuffer.SetAttribute ("PacketSize", UintegerValue (1400));
      fullBuffer.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      fullBuffer.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      fullBuffer.SetAttribute ("DataRate", DataRateValue (DataRate (rate)));
      clientApp.Add (fullBuffer.Install (remoteHost));
    }
  classLog.close ();
  if (clusterUes)
    {
      NS_LOG_UNCOND ("UE clusters left/right = " << nLeft << "/" << nRight
                                                << " rates=" << rateLeft << "/" << rateRight);
    }
  else
    {
      NS_LOG_UNCOND ("UE classes high/mid/low = " << nHigh << "/" << nMid << "/" << nLow);
    }

  GlobalValue::GetValueByName ("simTime", doubleValue);
  double simTime = doubleValue.Get ();
  sinkApp.Start (Seconds (0));
  clientApp.Start (MilliSeconds (50));
  clientApp.Stop (Seconds (simTime));

  g_slotLog.open ("asm-slot-alloc.csv");
  g_slotLog << "timeMs,frame,subframe,slot,dataSym,ctrlSym,expectedDataSym,overBudget,dlCtrl,ulCtrl"
            << std::endl;
  Config::ConnectWithoutContextFailSafe (
      "/NodeList/*/DeviceList/*/ComponentCarrierMap/*/MmWaveEnbMac/SchedulingTraceEnb",
      MakeCallback (&ReportAsmSched));

  if (enableTraces)
    {
      mmwaveHelper->EnableTraces ();
      Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
      lteHelper->Initialize ();
      lteHelper->EnablePhyTraces ();
      lteHelper->EnableMacTraces ();
    }

  Simulator::Schedule (MilliSeconds (100), &ForceMmWaveOnlyData, lteEnb, mcUeDevs);
  Simulator::Schedule (MilliSeconds (200), &ForceMmWaveOnlyData, lteEnb, mcUeDevs);
  Simulator::Schedule (MilliSeconds (250), &PrintUeAttachment, std::string ("UeAttachment.txt"),
                       ueNodes, mcUeDevs, lteEnb);

  g_tputLog.open ("asm-path-tput.csv");
  g_tputLog << "timeMs,asmModes,ltePdcpBytes,mmwPdcpBytes,lteKbps,mmwKbps,lteFrac,appRxDeltaBytes,"
               "appKbps,asmState,asmPowerW,dataSym,cellPower"
            << std::endl;
  g_ueSatLog.open ("ue-app-sat.csv");
  g_ueSatLog << "timeMs,imsi,appClass,appRate,rxKbps,offeredKbps,sat,servingNrCellId,useMmWave"
             << std::endl;
  static uint64_t lastLteBytes = 0;
  static uint64_t lastMmwBytes = 0;
  static uint64_t lastAppBytes = 0;
  Simulator::Schedule (MilliSeconds (80), &ConnectPdcpPathTraces, lteEnb);
  Simulator::Schedule (Seconds (tputPeriod), &ReportPathThroughput, lteEnb, sinkApp, tputPeriod,
                       &lastLteBytes, &lastMmwBytes, &lastAppBytes);

  Simulator::Stop (Seconds (simTime + 0.005));
  Simulator::Run ();

  if (g_asmStats.minDataSym == UINT32_MAX)
    {
      g_asmStats.minDataSym = 0;
    }
  double meanData = g_asmStats.nSlots > 0
                        ? static_cast<double> (g_asmStats.sumDataSym) / g_asmStats.nSlots
                        : 0.0;
  double dlCtrlFrac = g_asmStats.nSlots > 0
                          ? static_cast<double> (g_asmStats.nSlotsWithDlCtrl) / g_asmStats.nSlots
                          : 0.0;
  double ulCtrlFrac = g_asmStats.nSlots > 0
                          ? static_cast<double> (g_asmStats.nSlotsWithUlCtrl) / g_asmStats.nSlots
                          : 0.0;

  std::ofstream summary ("asm-summary.txt");
  summary << "asmMode," << unsigned (g_asmStats.asmMode) << std::endl;
  summary << "expectedDataSymbols," << g_asmStats.expectedDataSym << std::endl;
  summary << "nSlots," << g_asmStats.nSlots << std::endl;
  summary << "minDataSym," << g_asmStats.minDataSym << std::endl;
  summary << "maxDataSym," << g_asmStats.maxDataSym << std::endl;
  summary << "meanDataSym," << meanData << std::endl;
  summary << "nSlotsOverBudget," << g_asmStats.nSlotsOverBudget << std::endl;
  summary << "dlCtrlSlotFrac," << dlCtrlFrac << std::endl;
  summary << "ulCtrlSlotFrac," << ulCtrlFrac << std::endl;

  NS_LOG_UNCOND ("ASM summary: mode=" << unsigned (g_asmStats.asmMode) << " expectedData="
                                      << g_asmStats.expectedDataSym << " slots=" << g_asmStats.nSlots
                                      << " dataSym min/mean/max=" << g_asmStats.minDataSym << "/"
                                      << meanData << "/" << g_asmStats.maxDataSym
                                      << " overBudget=" << g_asmStats.nSlotsOverBudget
                                      << " dlCtrlFrac=" << dlCtrlFrac
                                      << " ulCtrlFrac=" << ulCtrlFrac);

  {
    std::ofstream appSum ("ue-app-rx-summary.txt");
    appSum << "imsi,rxBytesTotal,appAvgThroughputKbps" << std::endl;
    double activeSec = std::max (simTime - 0.050, 1e-9);
    uint64_t totalRx = 0;
    for (uint32_t i = 0; i < sinkApp.GetN (); ++i)
      {
        Ptr<PacketSink> sink = DynamicCast<PacketSink> (sinkApp.Get (i));
        Ptr<McUeNetDevice> ueDev = DynamicCast<McUeNetDevice> (mcUeDevs.Get (i));
        if (!sink || !ueDev)
          {
            continue;
          }
        uint64_t total = sink->GetTotalRx ();
        totalRx += total;
        uint64_t imsi = ueDev->GetImsi ();
        if (imsi == 0)
          {
            imsi = i + 1;
          }
        double kbps = total * 8.0 / 1e3 / activeSec;
        appSum << FormatImsi (imsi) << "," << total << "," << kbps << std::endl;
      }
    summary << "ueAppRxBytes," << totalRx << std::endl;
    summary << "ltePdcpBytes," << g_ltePdcpBytes << std::endl;
    summary << "mmwPdcpBytes," << g_mmwPdcpBytes << std::endl;
    uint64_t pathTotal = g_ltePdcpBytes + g_mmwPdcpBytes;
    double lteFrac = pathTotal > 0 ? static_cast<double> (g_ltePdcpBytes) / pathTotal : 0.0;
    summary << "ltePdcpFrac," << lteFrac << std::endl;
    NS_LOG_UNCOND ("UE app Rx total " << totalRx << " B over " << nUeNodes << " UEs");
    NS_LOG_UNCOND ("PDCP total LTE " << g_ltePdcpBytes << " B mmWave " << g_mmwPdcpBytes
                                    << " B lteFrac " << lteFrac);
  }

  if (g_tputLog.is_open ())
    {
      g_tputLog.close ();
    }
  if (g_ueSatLog.is_open ())
    {
      g_ueSatLog.close ();
    }

  if (g_slotLog.is_open ())
    {
      g_slotLog.close ();
    }
  Simulator::Destroy ();
  return 0;
}
