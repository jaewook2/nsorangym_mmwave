/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   Author: Marco Miozzo <marco.miozzo@cttc.es>
 *           Nicola Baldo  <nbaldo@cttc.es>
 *
 *   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
 *                         Sourjya Dutta <sdutta@nyu.edu>
 *                         Russell Ford <russell.ford@nyu.edu>
 *                         Menglei Zhang <menglei@nyu.edu>
 *
 *       Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                                Integration of Carrier Aggregation
 */

#include "mmwave-enb-net-device.h"

#include "encode_e2apv1.hpp"
#include "mmwave-net-device.h"
#include "mmwave-phy-mac-common.h"
#include "mmwave-ue-net-device.h"

#include <ns3/abort.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>

#include <cmath>
#include <ns3/config.h>
#include <ns3/double.h>
#include <ns3/enum.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/llc-snap-header.h>
#include <ns3/log.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-rlc-am.h>
#include <ns3/lte-rlc-um-lowlat.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/mmwave-component-carrier-enb.h>
#include <ns3/mmwave-indication-message-helper.h>
#include <ns3/node-list.h>
#include <ns3/node.h>
#include <ns3/packet-burst.h>
#include <ns3/packet.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/uinteger.h>

#include <cctype>
#include <exception>
#include <fstream>
#include <numeric>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWaveEnbNetDevice");

namespace mmwave
{

NS_OBJECT_ENSURE_REGISTERED(MmWaveEnbNetDevice);

/**
 * KPM Subscription Request callback.
 * This function is triggered whenever a RIC Subscription Request for
 * the KPM RAN Function is received.
 *
 * \param pdu request message
 */
void
MmWaveEnbNetDevice::KpmSubscriptionCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_DEBUG("\nReceived RIC Subscription Request, cellId= " << m_cellId << "\n");

    m_e2ReportParams = m_e2term->ProcessRicSubscriptionRequest(sub_req_pdu);
    m_e2Subscribed = true;
    NS_LOG_DEBUG("requestorId " << +m_e2ReportParams.requestorId << ", instanceId "
                                << +m_e2ReportParams.instanceId << ", ranFuncionId "
                                << +m_e2ReportParams.ranFuncionId << ", actionId "
                                << +m_e2ReportParams.actionId);

    if (!m_isReportingEnabled)
    {
        m_isReportingEnabled = true;
        Simulator::ScheduleWithContext(GetNode()->GetId(),
                                       Seconds(0),
                                       &MmWaveEnbNetDevice::BuildAndSendReportMessage,
                                       this,
                                       m_e2ReportParams);
    }
}

bool
MmWaveEnbNetDevice::IsE2Subscribed() const
{
    return m_e2Subscribed.load();
}

void
MmWaveEnbNetDevice::TurnOn(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc)
{
    m_rrc->SetSecondaryCellHandoverAllowedStatus(nodeId, enumModeEnergyBs::ON);
    SetCellState(enumModeEnergyBs::ON);
}

void
MmWaveEnbNetDevice::TurnIdle(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc)
{
    m_rrc->SetSecondaryCellHandoverAllowedStatus(nodeId, enumModeEnergyBs::Idle);
    SetCellState(enumModeEnergyBs::Idle);
}

void
MmWaveEnbNetDevice::TurnSleep(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc)
{
    m_rrc->SetSecondaryCellHandoverAllowedStatus(nodeId, enumModeEnergyBs::Sleep);
    m_rrc->EvictUsersFromSecondaryCell();
    SetCellState(enumModeEnergyBs::Sleep);
}

void
MmWaveEnbNetDevice::TurnOff(uint16_t nodeId, Ptr<LteEnbRrc> m_rrc)
{
    m_rrc->SetSecondaryCellHandoverAllowedStatus(nodeId, enumModeEnergyBs::OFF);
    m_rrc->EvictUsersFromSecondaryCell();
    SetCellState(enumModeEnergyBs::OFF);
}

bool
MmWaveEnbNetDevice::GetBsState()
{
    return m_CellState;
}

uint16_t
MmWaveEnbNetDevice::GetNUeGoodSinr()
{
    return m_nUeGoodSinr;
}

void
MmWaveEnbNetDevice::SetNUeGoodSinr(uint16_t value)
{
    m_nUeGoodSinr = value;
}

std::pair<double, double>
MmWaveEnbNetDevice::GetClosestUePos()
{
    return m_closestUEPos;
}

void
MmWaveEnbNetDevice::SetClosestUePos(std::pair<double, double> pos)
{
    m_closestUEPos = pos;
}

double
MmWaveEnbNetDevice::GetClosestUeTime()
{
    return m_closestUETime;
}

void
MmWaveEnbNetDevice::SetClosestUeTime(double time)
{
    m_closestUETime = time;
}

std::map<uint64_t, std::map<uint16_t, long double>>
MmWaveEnbNetDevice::Getl3sinrMap()
{
    return m_l3sinrMap;
}

void
MmWaveEnbNetDevice::SetCellState(enumModeEnergyBs value)
{
    m_CellState = value;
}

uint32_t
MmWaveEnbNetDevice::GetMacVolumeCellSpecific()
{
    return m_macVolumeCellSpecific;
}

uint32_t
MmWaveEnbNetDevice::GetMacPduCellSpecific()
{
    return m_macPduCellSpecific;
}

void
MmWaveEnbNetDevice::SetTurnOffTime(double value)
{
    m_turnOffTime = value;
}

double
MmWaveEnbNetDevice::GetTurnOffTime()
{
    return m_turnOffTime;
}

TypeId MmWaveEnbNetDevice::GetTypeId ()
{
    static TypeId tid =
        TypeId("ns3::MmWaveEnbNetDevice")
            .SetParent<MmWaveNetDevice>()
            .AddConstructor<MmWaveEnbNetDevice>()
            .AddAttribute("LteEnbComponentCarrierManager",
                          "The ComponentCarrierManager associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_componentCarrierManager),
                          MakePointerChecker<LteEnbComponentCarrierManager>())
            .AddAttribute("LteEnbRrc",
                          "The RRC layer associated with the ENB",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_rrc),
                          MakePointerChecker<LteEnbRrc>())
            .AddAttribute("E2Termination",
                          "The E2 termination object associated to this node",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::SetE2Termination,
                                              &MmWaveEnbNetDevice::GetE2Termination),
                          MakePointerChecker<E2Termination>())
            .AddAttribute("CellId",
                          "Cell Identifier",
                          UintegerValue(0),
                          MakeUintegerAccessor(&MmWaveEnbNetDevice::m_cellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("AsmMode",
                          "duty_mode data symbols: 1=100%, 2=75%, 3=50%, 4=25%, 5=0% of data "
                          "symbols. 6=TurnOff (HO barred, evict to other mmWave or LTE). "
                          "When AsmStateMachine is true this is the ASM target: "
                          "1=Active, 2=ASM1, 3=ASM2, 4=ASM3, 5=ASM4, 6=OFF.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&MmWaveEnbNetDevice::SetAsmMode,
                                               &MmWaveEnbNetDevice::GetAsmMode),
                          MakeUintegerChecker<uint8_t>(1, 6))
            .AddAttribute("AsmStateMachine",
                          "If true, AsmMode is a real ASM target and transitions follow "
                          "component on/off times. If false, AsmMode is duty_mode.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_asmStateMachine),
                          MakeBooleanChecker())
            .AddAttribute("AsmPaOffTime",
                          "Time to turn the PA off. Default is SM1 deactivation "
                          "(Salem et al., 2019; half of a 71 us OFDM symbol).",
                          TimeValue(Seconds(35.5e-6)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_paOff),
                          MakeTimeChecker())
            .AddAttribute("AsmRfOffTime",
                          "Time to turn RF off. Default is the SM2 analog front-end "
                          "deactivation, 0.5 ms.",
                          TimeValue(Seconds(0.5e-3)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_rfOff),
                          MakeTimeChecker())
            .AddAttribute("AsmBbOffTime",
                          "Time to turn the baseband off. Default is the SM3 digital "
                          "baseband deactivation, 5 ms.",
                          TimeValue(Seconds(5e-3)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_bbOff),
                          MakeTimeChecker())
            .AddAttribute("AsmControlToMinimalTime",
                          "Time to reduce Control to Minimal. Default is the SM4 "
                          "standby deactivation, 0.5 s.",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_controlToMinimal),
                          MakeTimeChecker())
            .AddAttribute("AsmMinimalOffTime",
                          "Time for Minimal to disappear into OFF. The four-level ASM "
                          "model stops at standby; this reuses that 0.5 s class.",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_minimalOff),
                          MakeTimeChecker())
            .AddAttribute("AsmOffToMinimalTime",
                          "Time to raise OFF to Minimal. Same 0.5 s class as "
                          "AsmMinimalOffTime.",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_offToMinimal),
                          MakeTimeChecker())
            .AddAttribute("AsmMinimalToControlTime",
                          "Time to raise Minimal to Control. Default is the SM4 "
                          "activation, 0.5 s.",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_minimalToControl),
                          MakeTimeChecker())
            .AddAttribute("AsmBbOnTime",
                          "Time to turn the baseband on. Default is the SM3 "
                          "activation, 5 ms.",
                          TimeValue(Seconds(5e-3)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_bbOn),
                          MakeTimeChecker())
            .AddAttribute("AsmRfOnTime",
                          "Time to turn RF on. Default is the SM2 activation, 0.5 ms.",
                          TimeValue(Seconds(0.5e-3)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_rfOn),
                          MakeTimeChecker())
            .AddAttribute("AsmPaOnTime",
                          "Time to turn the PA on. Default is the SM1 activation, "
                          "35.5 us.",
                          TimeValue(Seconds(35.5e-6)),
                          MakeTimeAccessor(&MmWaveEnbNetDevice::m_paOn),
                          MakeTimeChecker())
            .AddAttribute("AsmRfPower",
                          "Operating power of RF in watts. Pico SM1 minus SM2 "
                          "(Onireti et al., PIMRC 2017).",
                          DoubleValue(1.142),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_rfPower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmBbPower",
                          "Operating power of the baseband in watts. Pico SM2 minus SM3.",
                          DoubleValue(0.067),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_bbPower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmControlPower",
                          "Operating power of Control in watts. Pico SM3 total, "
                          "since Control replaces Minimal.",
                          DoubleValue(0.301),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_controlPower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmMinimalPower",
                          "Operating power of Minimal in watts. Pico SM4 total.",
                          DoubleValue(0.238),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_minimalPower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmPaWakePower",
                          "Power while the PA is turning on, in watts. "
                          "Twice the 30 dBm TxPower reference.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_paWakePower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmRfWakePower",
                          "Power while RF is turning on, in watts. Twice AsmRfPower.",
                          DoubleValue(2.284),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_rfWakePower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmBbWakePower",
                          "Power while the baseband is turning on, in watts. Twice AsmBbPower.",
                          DoubleValue(0.134),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_bbWakePower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmControlWakePower",
                          "Power while Control is turning on, in watts. Twice AsmControlPower.",
                          DoubleValue(0.602),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_controlWakePower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("AsmMinimalWakePower",
                          "Power while Minimal is turning on, in watts. Twice AsmMinimalPower.",
                          DoubleValue(0.476),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_minimalWakePower),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("BasicCellId",
                          "Basic cell ID. This is needed to properly loop over neighbors.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&MmWaveEnbNetDevice::m_basicCellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("E2PdcpCalculator",
                          "The PDCP calculator object for E2 reporting",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_e2PdcpStatsCalculator),
                          MakePointerChecker<MmWaveBearerStatsCalculator>())
            .AddAttribute("E2Periodicity",
                          "Periodicity of E2 reporting (value in seconds)",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_e2Periodicity),
                          MakeDoubleChecker<double>())
            .AddAttribute("E2RlcCalculator",
                          "The RLC calculator object for E2 reporting",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_e2RlcStatsCalculator),
                          MakePointerChecker<MmWaveBearerStatsCalculator>())
            .AddAttribute("E2DuCalculator",
                          "The DU calculator object for E2 reporting",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_e2DuCalculator),
                          MakePointerChecker<MmWavePhyTrace>())
            .AddAttribute("EnableCuUpReport",
                          "If true, include the CU-UP report. "
                          "Appended to cu-up-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_sendCuUp),
                          MakeBooleanChecker())
            .AddAttribute("EnableCuCpReport",
                          "If true, include the CU-CP report. "
                          "Appended to cu-cp-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_sendCuCp),
                          MakeBooleanChecker())
            .AddAttribute("EnableDuReport",
                          "If true, include the DU report. "
                          "Appended to du-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_sendDu),
                          MakeBooleanChecker())
            .AddAttribute("ReducedPmValues",
                          "If true, send only a subset of pmValues",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_reducedPmValues),
                          MakeBooleanChecker())
            .AddAttribute("EnableGYMFileLogging",
                          "If true, write the enabled CU-UP, CU-CP, and DU reports to csv files.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_enableGymFileLogging),
                          MakeBooleanChecker())
            .AddAttribute(
                "ControlFileNameAsm",
                "[nsoran-asm] CSV of runtime ASM updates. "
                "timestampMs,cellId,asmMode (cellId 0 = this cell) or timestampMs,asmMode. "
                "Empty disables runtime ASM control.",
                StringValue(""),
                MakeStringAccessor(&MmWaveEnbNetDevice::m_asmControlFilename),
                MakeStringChecker())
            .AddAttribute("ScheduleAsmControl",
                          "[nsoran-asm] If true, timestamps are absolute sim time in ms and the "
                          "file is read once. If false, apply immediately and poll every "
                          "AsmControlPeriodicity seconds.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_scheduleAsmControl),
                          MakeBooleanChecker())
            .AddAttribute("FlushAsmControlFile",
                          "[nsoran-asm] If true and ScheduleAsmControl is false, truncate the CSV "
                          "after each read (Gym-style). Keep false when several gNBs share one file.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&MmWaveEnbNetDevice::m_flushAsmControlFile),
                          MakeBooleanChecker())
            .AddAttribute("AsmControlPeriodicity",
                          "[nsoran-asm] Poll period (s) when ScheduleAsmControl is false.",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&MmWaveEnbNetDevice::m_asmControlPeriodicity),
                          MakeDoubleChecker<double>(0.001, 10.0));
    return tid;
}

MmWaveEnbNetDevice::MmWaveEnbNetDevice()
    //: m_cellId(0),
    // m_Bandwidth (72),
    // m_Earfcn(1),
    : m_componentCarrierManager(0),
      m_isConfigured(false),
      m_isReportingEnabled(false),
      m_reducedPmValues(false),
      m_enableGymFileLogging(false),
      m_e2Subscribed(false),
      m_cuUpFileName(),
      m_cuCpFileName(),
      m_duFileName()
{
    NS_LOG_FUNCTION(this);
}

MmWaveEnbNetDevice::~MmWaveEnbNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbNetDevice::DoInitialize(void)
{
    NS_LOG_FUNCTION(this);
    m_isConstructed = true;
    UpdateConfig();
    for (auto it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
    {
        it->second->Initialize();
    }
    m_rrc->Initialize();
    m_componentCarrierManager->Initialize();
    ApplyAsmToSchedulers();
    if (m_asmStateMachine)
    {
        m_asmCurrent = m_asmMode;
        m_asmTarget = m_asmCurrent;
        m_asmPhase = 0;
        AsmInstallSlotHook();
    }
    if (m_asmMode == 6)
    {
        ApplyAsmTurnOff();
    }
    if (!m_asmControlFilename.empty())
    {
        Simulator::Schedule(MilliSeconds(1), &MmWaveEnbNetDevice::ReadAsmControlFile, this);
        Config::ConnectFailSafe(
            "/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/GymControlReady",
            MakeBoundCallback(&MmWaveEnbNetDevice::RecvGymControlReady, this));
    }

    if (m_sendCuCp)
    {
        // connect to callback
        Config::ConnectFailSafe(
            "/NodeList/*/DeviceList/*/LteEnbRrc/NotifyMmWaveSinr",
            MakeBoundCallback(&MmWaveEnbNetDevice::RegisterNewSinrReadingCallback, this));
    }
}

void
MmWaveEnbNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);

    m_rrc->Dispose();
    m_rrc = 0;

    m_componentCarrierManager->Dispose();
    m_componentCarrierManager = 0;
    // MmWaveComponentCarrierEnb::DoDispose() will call DoDispose
    // of its PHY, MAC, FFR and scheduler instance
    for (uint32_t i = 0; i < m_ccMap.size(); i++)
    {
        m_ccMap.at(i)->Dispose();
        m_ccMap.at(i) = 0;
    }

    MmWaveNetDevice::DoDispose();
}

void
MmWaveEnbNetDevice::RegisterNewSinrReadingCallback(Ptr<MmWaveEnbNetDevice> netDev,
                                                   std::string context,
                                                   uint64_t imsi,
                                                   uint16_t cellId,
                                                   long double sinr)
{
    netDev->RegisterNewSinrReading(imsi, cellId, sinr);
}

void
MmWaveEnbNetDevice::RegisterNewSinrReading(uint64_t imsi, uint16_t cellId, long double sinr)
{
    // check if the imsi is connected to this DU
    auto ueMap = m_rrc->GetUeMap();
    bool imsiFound = false;

    for (auto ue : ueMap)
    {
        if (ue.second->GetImsi() == imsi)
        {
            imsiFound = true;
            break;
        }
    }

    if (imsiFound)
    {
        // we only need to save the last value, so we erase if exists already a value and save the
        // new one
        m_l3sinrMap[imsi][cellId] = sinr;
        NS_LOG_LOGIC(Simulator::Now().GetSeconds()
                     << " enbdev " << m_cellId << " UE " << imsi << " report for " << cellId
                     << " SINR " << m_l3sinrMap[imsi][cellId]);
    }
}

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy(void) const
{
    NS_LOG_FUNCTION(this);
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(0))->GetPhy();
}

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy(uint8_t index)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(index))->GetPhy();
}

uint16_t
MmWaveEnbNetDevice::GetCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_cellId;
}

bool
MmWaveEnbNetDevice::HasCellId(uint16_t cellId) const
{
    for (auto& it : m_ccMap)
    {
        if (DynamicCast<MmWaveComponentCarrierEnb>(it.second)->GetCellId() == cellId)
        {
            return true;
        }
    }
    return false;
}

uint8_t
MmWaveEnbNetDevice::GetBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_Bandwidth;
}

void
MmWaveEnbNetDevice::SetBandwidth(uint8_t bw)
{
    NS_LOG_FUNCTION(this << bw);
    m_Bandwidth = bw;
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac(void)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(0))->GetMac();
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac(uint8_t index)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(index))->GetMac();
}

void
MmWaveEnbNetDevice::SetRrc(Ptr<LteEnbRrc> rrc)
{
    m_rrc = rrc;
}

Ptr<LteEnbRrc>
MmWaveEnbNetDevice::GetRrc(void)
{
    return m_rrc;
}

bool
MmWaveEnbNetDevice::DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                        protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber << ", only IPv4/IPv6 is supported");
    return m_rrc->SendData(packet);
}

void
MmWaveEnbNetDevice::UpdateConfig(void)
{
    NS_LOG_FUNCTION(this);

    if (m_isConstructed)
    {
        if (!m_isConfigured)
        {
            NS_LOG_LOGIC(this << " Configure cell " << m_cellId);
            // we have to make sure that this function is called only once
            // m_rrc->ConfigureCell (m_Bandwidth, m_Bandwidth, m_Earfcn, m_Earfcn, m_cellId);
            NS_ASSERT(!m_ccMap.empty());

            // create the MmWaveComponentCarrierConf map used for the RRC setup
            std::map<uint8_t, LteEnbRrc::MmWaveComponentCarrierConf> ccConfMap;
            for (auto it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
            {
                Ptr<MmWaveComponentCarrierEnb> ccEnb =
                    DynamicCast<MmWaveComponentCarrierEnb>(it->second);
                LteEnbRrc::MmWaveComponentCarrierConf ccConf;
                ccConf.m_ccId = ccEnb->GetConfigurationParameters()->GetCcId();
                ccConf.m_cellId = ccEnb->GetCellId();
                ccConf.m_bandwidth = ccEnb->GetBandwidthInRb();

                ccConfMap[it->first] = ccConf;
            }

            m_rrc->ConfigureCell(ccConfMap);

            // trigger E2Termination activation for when the simulation starts
            // schedule at start time
            if (m_e2term)
            {
                NS_LOG_DEBUG("E2sim start in cell " << m_cellId << " gym file logging "
                                                    << m_enableGymFileLogging);

                Simulator::Schedule(MicroSeconds(0), &E2Termination::Start, m_e2term);
                if (m_enableGymFileLogging)
                {
                    m_cuUpFileName = "cu-up-cell-" + std::to_string(m_cellId) + ".txt";
                    std::ofstream csv{};
                    csv.open(m_cuUpFileName.c_str());
                    csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl(cellAverageLatency),"
                           "m_pDCPBytesUL(0),"
                           "m_pDCPBytesDL(cellDlTxVolume),"
                           "DRB.PdcpSduVolumeDl_Filter.UEID(txBytes),"
                           "Tot.PdcpSduNbrDl.UEID(txDlPackets),"
                           "DRB.PdcpSduBitRateDl.UEID(pdcpThroughput),"
                           "DRB.PdcpSduDelayDl.UEID(pdcpLatency),"
                           "txPdcpPduLteRlc,txPdcpPduBytesLteRlc,"
                           "QosFlow.PdcpPduVolumeDL_Filter.UEID(txPdcpPduBytesNrRlc),"
                           "DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
                    csv.close();

                    m_cuCpFileName = "cu-cp-cell-" + std::to_string(m_cellId) + ".txt";
                    csv.open(m_cuCpFileName.c_str());
                    csv << "timestamp,ueImsiComplete,numActiveUes,RRC.ConnMean,DRB.EstabSucc.5QI."
                           "UEID (numDrb),"
                           "DRB.RelActNbr.5QI.UEID (0),L3 serving Id(m_cellId),UE (imsi),L3 "
                           "serving SINR,"
                           "L3 serving SINR 3gpp,"
                           "L3 neigh Id 1 (cellId),L3 neigh SINR 1,L3 neigh SINR 3gpp 1 "
                           "(convertedSinr),"
                           "L3 neigh Id 2 (cellId),L3 neigh SINR 2,L3 neigh SINR 3gpp 2 "
                           "(convertedSinr),"
                           "L3 neigh Id 3 (cellId),L3 neigh SINR 3,L3 neigh SINR 3gpp 3 "
                           "(convertedSinr),"
                           "L3 neigh Id 4 (cellId),L3 neigh SINR 4,L3 neigh SINR 3gpp 4 "
                           "(convertedSinr),"
                           "L3 neigh Id 5 (cellId),L3 neigh SINR 5,L3 neigh SINR 3gpp 5 "
                           "(convertedSinr),"
                           "L3 neigh Id 6 (cellId),L3 neigh SINR 6,L3 neigh SINR 3gpp 6 "
                           "(convertedSinr),"
                           "L3 neigh Id 7 (cellId),L3 neigh SINR 7,L3 neigh SINR 3gpp 7 "
                           "(convertedSinr),"
                           "L3 neigh Id 8 (cellId),L3 neigh SINR 8,L3 neigh SINR 3gpp 8 "
                           "(convertedSinr)"
                           "\n";
                    csv.close();

                    m_duFileName = "du-cell-" + std::to_string(m_cellId) + ".txt";
                    csv.open(m_duFileName.c_str());

                    std::string header_csv =
                        "timestamp,ueImsiComplete,plmId,nrCellId,dlAvailablePrbs,"
                        "ulAvailablePrbs,qci,dlPrbUsage,ulPrbUsage";

                    std::string cell_header =
                        "TB.TotNbrDl.1,TB.TotNbrDlInitial,TB.TotNbrDlInitial.Qpsk,"
                        "TB.TotNbrDlInitial.16Qam,"
                        "TB.TotNbrDlInitial.64Qam,RRU.PrbUsedDl,TB.ErrTotalNbrDl.1,"
                        "QosFlow.PdcpPduVolumeDL_Filter,CARR.PDSCHMCSDist.Bin1,"
                        "CARR.PDSCHMCSDist.Bin2,"
                        "CARR.PDSCHMCSDist.Bin3,CARR.PDSCHMCSDist.Bin4,CARR.PDSCHMCSDist.Bin5,"
                        "CARR.PDSCHMCSDist.Bin6,L1M.RS-SINR.Bin34,L1M.RS-SINR.Bin46,"
                        "L1M.RS-SINR.Bin58,"
                        "L1M.RS-SINR.Bin70,L1M.RS-SINR.Bin82,L1M.RS-SINR.Bin94,L1M.RS-SINR.Bin127,"
                        "DRB.BufferSize.Qos,DRB.MeanActiveUeDl";

                    std::string ue_header =
                        "TB.TotNbrDl.1.UEID,TB.TotNbrDlInitial.UEID,TB.TotNbrDlInitial.Qpsk.UEID,"
                        "TB.TotNbrDlInitial.16Qam.UEID,TB.TotNbrDlInitial.64Qam.UEID,"
                        "TB.ErrTotalNbrDl.1.UEID,"
                        "QosFlow.PdcpPduVolumeDL_Filter.UEID,RRU.PrbUsedDl.UEID,"
                        "CARR.PDSCHMCSDist.Bin1.UEID,"
                        "CARR.PDSCHMCSDist.Bin2.UEID,CARR.PDSCHMCSDist.Bin3.UEID,"
                        "CARR.PDSCHMCSDist.Bin4.UEID,"
                        "CARR.PDSCHMCSDist.Bin5.UEID,"
                        "CARR.PDSCHMCSDist.Bin6.UEID,L1M.RS-SINR.Bin34.UEID,"
                        "L1M.RS-SINR.Bin46.UEID,"
                        "L1M.RS-SINR.Bin58.UEID,L1M.RS-SINR.Bin70.UEID,L1M.RS-SINR.Bin82.UEID,"
                        "L1M.RS-SINR.Bin94.UEID,L1M.RS-SINR.Bin127.UEID,DRB.BufferSize.Qos.UEID,"
                        "DRB.UEThpDl.UEID,DRB.UEThpDlPdcpBased.UEID";

                    std::string asm_header = "ASM.Mode,ASM.Phase,ASM.Target,ASM.PowerW";

                    csv << header_csv + "," + cell_header + "," + ue_header + "," + asm_header +
                               "\n";
                    csv.close();
                    Simulator::Schedule(MicroSeconds(800),
                                        &MmWaveEnbNetDevice::BuildAndSendReportMessage,
                                        this,
                                        E2Termination::RicSubscriptionRequest_rval_s{});
                    m_isReportingEnabled = true;
                }

                // Regardless the offline or online mode for reporting the files, we always want to
                // register the mean of RRC UEs
                Simulator::Schedule(MilliSeconds(10), &MmWaveEnbNetDevice::SaveActiveUes, this);
            }
            m_isConfigured = true;
        }

        // m_rrc->SetCsgId (m_csgId, m_csgIndication);
    }
    else
    {
        /*
         * Lower layers are not ready yet, so do nothing now and expect
         * ``DoInitialize`` to re-invoke this function.
         */
    }
}

void
MmWaveEnbNetDevice::SetCcMap(std::map<uint8_t, Ptr<MmWaveComponentCarrier>> ccm)
{
    NS_ASSERT_MSG(!m_isConfigured, "attempt to set CC map after configuration");
    m_ccMap = ccm;
    ApplyAsmToSchedulers();
    if (m_asmStateMachine)
    {
        AsmInstallSlotHook();
    }
}

void
MmWaveEnbNetDevice::SetAsmMode(uint8_t mode)
{
    NS_ABORT_MSG_IF(mode < 1 || mode > 6, "ASM mode must be 1..6");
    if (m_asmStateMachine)
    {
        AsmCommandTarget(mode);
        return;
    }
    if (mode == m_asmMode)
    {
        return;
    }
    uint8_t oldMode = m_asmMode;
    m_asmMode = mode;
    ApplyAsmToSchedulers();
    if (mode == 6 && oldMode != 6)
    {
        ApplyAsmTurnOff();
    }
    else if (oldMode == 6 && mode != 6)
    {
        ApplyAsmTurnOn();
    }
    if (oldMode != mode)
    {
        LogAsmModeChange(oldMode, mode);
    }
}

Ptr<LteEnbRrc>
MmWaveEnbNetDevice::FindMasterLteRrc() const
{
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        for (uint32_t i = 0; i < node->GetNDevices(); ++i)
        {
            Ptr<LteEnbNetDevice> lte = DynamicCast<LteEnbNetDevice>(node->GetDevice(i));
            if (lte && lte->GetRrc())
            {
                return lte->GetRrc();
            }
        }
    }
    return nullptr;
}

void
MmWaveEnbNetDevice::ApplyAsmTurnOff()
{
    if (m_cellId == 0)
    {
        return;
    }
    Ptr<LteEnbRrc> lteRrc = FindMasterLteRrc();
    if (!lteRrc)
    {
        NS_LOG_WARN("ASM 6: master LTE RRC not found for cell " << m_cellId);
        return;
    }
    NS_LOG_UNCOND(Simulator::Now().GetMilliSeconds()
                  << " ms mmWave cell " << m_cellId << " ASM 6 TurnOff: bar HO and evict");
    lteRrc->SetSecondaryCellHandoverAllowedStatus(m_cellId, false);
    lteRrc->EvictUsersFromSecondaryCell();
    SetCellState(enumModeEnergyBs::OFF);
}

void
MmWaveEnbNetDevice::ApplyAsmTurnOn()
{
    if (m_cellId == 0)
    {
        return;
    }
    Ptr<LteEnbRrc> lteRrc = FindMasterLteRrc();
    if (!lteRrc)
    {
        NS_LOG_WARN("ASM on: master LTE RRC not found for cell " << m_cellId);
        return;
    }
    NS_LOG_UNCOND(Simulator::Now().GetMilliSeconds()
                  << " ms mmWave cell " << m_cellId << " leave ASM 6: allow HO and resume LTE UEs");
    lteRrc->SetSecondaryCellHandoverAllowedStatus(m_cellId, true);
    lteRrc->ResumeUesFromLteToMmWaveCell(m_cellId);
    SetCellState(enumModeEnergyBs::ON);
}

void
MmWaveEnbNetDevice::LogAsmModeChange(uint8_t oldMode, uint8_t newMode)
{
    int dataSym = -1;
    auto ccMap = GetCcMap();
    if (!ccMap.empty())
    {
        Ptr<MmWaveComponentCarrierEnb> cc =
            DynamicCast<MmWaveComponentCarrierEnb>(ccMap.begin()->second);
        if (cc && cc->GetMacScheduler())
        {
            dataSym = cc->GetMacScheduler()->GetAvailableDataSymbols();
        }
    }
    NS_LOG_UNCOND(Simulator::Now().GetMilliSeconds()
                  << " ms mmWave cell " << m_cellId << " ASM " << unsigned(oldMode) << " -> "
                  << unsigned(newMode) << " dataSymbols=" << dataSym);
    static std::ofstream logFile;
    if (!logFile.is_open())
    {
        logFile.open("asm-mode-changes.txt", std::ios_base::out | std::ios_base::trunc);
        logFile << "timeMs,cellId,oldMode,newMode,dataSymbols" << std::endl;
    }
    logFile << Simulator::Now().GetMilliSeconds() << "," << m_cellId << "," << unsigned(oldMode)
            << "," << unsigned(newMode) << "," << dataSym << std::endl;
    logFile.flush();
}

void
MmWaveEnbNetDevice::ReadAsmControlFile()
{
    if (m_asmControlFilename.empty())
    {
        return;
    }
    ApplyAsmControlFile();
    if (!m_scheduleAsmControl)
    {
        if (m_flushAsmControlFile)
        {
            std::ofstream csvDelete(m_asmControlFilename.c_str(), std::ios_base::trunc);
        }
        Simulator::Schedule(Seconds(m_asmControlPeriodicity),
                            &MmWaveEnbNetDevice::ReadAsmControlFile,
                            this);
    }
}

void
MmWaveEnbNetDevice::ApplyAsmControlFile()
{
    std::ifstream csv(m_asmControlFilename.c_str());
    if (!csv.is_open())
    {
        NS_LOG_WARN("ASM control file not open yet: " << m_asmControlFilename);
        return;
    }

    std::string line;
    uint16_t lastPollMode = 0;
    bool havePollMode = false;
    while (std::getline(csv, line))
    {
        if (line.empty())
        {
            continue;
        }
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
        {
            line.pop_back();
        }
        if (line.empty() || !std::isdigit(static_cast<unsigned char>(line[0])))
        {
            continue;
        }

        std::stringstream lineStream(line);
        std::string field;
        if (!std::getline(lineStream, field, ','))
        {
            continue;
        }
        long long timestampMs = 0;
        try
        {
            timestampMs = std::stoll(field);
        }
        catch (const std::exception&)
        {
            continue;
        }

        if (!std::getline(lineStream, field, ','))
        {
            continue;
        }
        uint16_t cellId = m_cellId;
        uint16_t mode = 0;
        std::string third;
        try
        {
            if (std::getline(lineStream, third, ',') && !third.empty())
            {
                cellId = static_cast<uint16_t>(std::stoi(field));
                mode = static_cast<uint16_t>(std::stoi(third));
            }
            else
            {
                mode = static_cast<uint16_t>(std::stoi(field));
            }
        }
        catch (const std::exception&)
        {
            continue;
        }

        if (cellId != 0 && cellId != m_cellId)
        {
            continue;
        }
        if (mode < 1 || mode > 6)
        {
            NS_LOG_ERROR("Invalid ASM mode " << mode << " in " << m_asmControlFilename);
            continue;
        }

        NS_LOG_INFO("ASM command ts " << timestampMs << " cell " << m_cellId << " mode " << mode);
        if (m_scheduleAsmControl)
        {
            Time target = MilliSeconds(timestampMs);
            Time now = Simulator::Now();
            if (target <= now)
            {
                SetAsmMode(static_cast<uint8_t>(mode));
            }
            else
            {
                Simulator::Schedule(target - now,
                                    &MmWaveEnbNetDevice::SetAsmMode,
                                    this,
                                    static_cast<uint8_t>(mode));
            }
        }
        else
        {
            lastPollMode = mode;
            havePollMode = true;
        }
    }
    if (havePollMode)
    {
        SetAsmMode(static_cast<uint8_t>(lastPollMode));
    }
}

uint8_t
MmWaveEnbNetDevice::GetAsmMode() const
{
    return m_asmStateMachine ? m_asmCurrent : m_asmMode;
}

uint8_t
MmWaveEnbNetDevice::GetAsmPhase() const
{
    return m_asmPhase;
}

uint8_t
MmWaveEnbNetDevice::GetAsmTarget() const
{
    return m_asmStateMachine ? m_asmTarget : GetAsmMode();
}

double
MmWaveEnbNetDevice::ConsumeAveragePowerW(double periodSec)
{
    AsmTouchEnergy();
    double average = periodSec > 0.0 ? m_energyJ / periodSec : m_cachedPower;
    m_energyJ = 0.0;
    m_lastAveragePowerW = average;
    return average;
}

double
MmWaveEnbNetDevice::GetLastAveragePowerW() const
{
    return m_lastAveragePowerW;
}

bool
MmWaveEnbNetDevice::LogsKpmToFile() const
{
    return m_enableGymFileLogging;
}

void
MmWaveEnbNetDevice::AsmCommandTarget(uint8_t mode)
{
    if (m_asmPhase != 0)
    {
        NS_LOG_INFO("Ignore ASM target " << unsigned(mode) << " on cell " << m_cellId
                                         << " while phase " << unsigned(m_asmPhase));
        return;
    }
    if (mode == m_asmCurrent)
    {
        return;
    }
    int fromDepth = static_cast<int>(m_asmCurrent) - 1;
    int toDepth = static_cast<int>(mode) - 1;
    m_asmTarget = mode;
    m_asmPhase = toDepth > fromDepth ? 1 : 2;
    m_asmTransStart = Simulator::Now();
    m_asmTransEnd = m_asmTransStart + AsmTransitionDuration(fromDepth, toDepth);
    NS_LOG_UNCOND(Simulator::Now().GetMilliSeconds()
                  << " ms mmWave cell " << m_cellId << " ASM target " << unsigned(m_asmCurrent)
                  << " -> " << unsigned(mode) << " phase " << unsigned(m_asmPhase)
                  << " delayMs " << m_asmTransEnd.GetMilliSeconds() - m_asmTransStart.GetMilliSeconds());
    // Data and radiated power follow the new phase from this instant. Deactivating
    // closes the gate now; Waking stays closed until completion opens it.
    AsmTouchEnergy();
    AsmApplyDataGate();
    AsmScheduleEnergySamples();
}

void
MmWaveEnbNetDevice::AsmOnSlotStart()
{
    if (!m_asmStateMachine)
    {
        return;
    }
    Time now = Simulator::Now();
    if (m_slotSeen && m_lastSlotSeen == now)
    {
        return;
    }
    m_slotSeen = true;
    m_lastSlotSeen = now;
    AsmTouchEnergy();
    if (m_asmPhase != 0 && now >= m_asmTransEnd)
    {
        AsmCompleteTransition();
        AsmTouchEnergy();
    }
    AsmApplyDataGate();
}

void
MmWaveEnbNetDevice::AsmInstallSlotHook()
{
    if (!m_asmStateMachine)
    {
        return;
    }
    for (auto& it : m_ccMap)
    {
        Ptr<MmWaveComponentCarrierEnb> cc = DynamicCast<MmWaveComponentCarrierEnb>(it.second);
        if (cc && cc->GetMac())
        {
            cc->GetMac()->SetSlotStartCallback(
                MakeCallback(&MmWaveEnbNetDevice::AsmOnSlotStart, this));
        }
    }
    AsmApplyDataGate();
}

void
MmWaveEnbNetDevice::AsmApplyDataGate()
{
    // Data only in stable Active. Deactivating closes it at the command.
    // Waking into Active stays closed until that transition completes.
    bool allow = m_asmCurrent == 1 && m_asmPhase == 0;
    for (auto& it : m_ccMap)
    {
        Ptr<MmWaveComponentCarrierEnb> cc = DynamicCast<MmWaveComponentCarrierEnb>(it.second);
        if (cc && cc->GetMacScheduler())
        {
            cc->GetMacScheduler()->SetAsmDataGate(true, allow);
        }
    }
}

void
MmWaveEnbNetDevice::AsmCompleteTransition()
{
    uint8_t oldMode = m_asmCurrent;
    uint8_t newMode = m_asmTarget;
    m_asmCurrent = newMode;
    m_asmMode = newMode;
    m_asmPhase = 0;
    if (newMode == 6 && oldMode != 6)
    {
        ApplyAsmTurnOff();
    }
    else if (oldMode == 6 && newMode != 6)
    {
        ApplyAsmTurnOn();
    }
    AsmApplyDataGate();
    LogAsmModeChange(oldMode, newMode);
}

void
MmWaveEnbNetDevice::AsmTouchEnergy()
{
    if (!m_asmStateMachine)
    {
        return;
    }
    Time now = Simulator::Now();
    if (!m_energyInit)
    {
        m_energyTime = now;
        m_cachedPower = AsmPowerW();
        m_energyInit = true;
        return;
    }
    if (now < m_energyTime)
    {
        return;
    }
    m_energyJ += m_cachedPower * (now - m_energyTime).GetSeconds();
    m_energyTime = now;
    m_cachedPower = AsmPowerW();
}

void
MmWaveEnbNetDevice::AsmScheduleEnergySamples()
{
    Time cursor = m_asmTransStart;
    int fromDepth = static_cast<int>(m_asmCurrent) - 1;
    int toDepth = static_cast<int>(m_asmTarget) - 1;
    auto scheduleAt = [this](Time when) {
        Time now = Simulator::Now();
        if (when > now)
        {
            Simulator::Schedule(when - now, &MmWaveEnbNetDevice::AsmTouchEnergy, this);
        }
    };
    if (toDepth > fromDepth)
    {
        for (int boundary = fromDepth; boundary < toDepth; ++boundary)
        {
            cursor += AsmOffDuration(boundary);
            scheduleAt(cursor);
        }
    }
    else
    {
        for (int boundary = fromDepth - 1; boundary >= toDepth; --boundary)
        {
            cursor += AsmOnDuration(boundary);
            scheduleAt(cursor);
        }
    }
}

Time
MmWaveEnbNetDevice::AsmOffDuration(int boundary) const
{
    switch (boundary)
    {
    case 0:
        return m_paOff;
    case 1:
        return m_rfOff;
    case 2:
        return m_bbOff;
    case 3:
        return m_controlToMinimal;
    case 4:
        return m_minimalOff;
    default:
        return Seconds(0);
    }
}

Time
MmWaveEnbNetDevice::AsmOnDuration(int boundary) const
{
    switch (boundary)
    {
    case 0:
        return m_paOn;
    case 1:
        return m_rfOn;
    case 2:
        return m_bbOn;
    case 3:
        return m_minimalToControl;
    case 4:
        return m_offToMinimal;
    default:
        return Seconds(0);
    }
}

Time
MmWaveEnbNetDevice::AsmTransitionDuration(int fromDepth, int toDepth) const
{
    Time total = Seconds(0);
    if (toDepth > fromDepth)
    {
        for (int boundary = fromDepth; boundary < toDepth; ++boundary)
        {
            total += AsmOffDuration(boundary);
        }
    }
    else
    {
        for (int boundary = fromDepth - 1; boundary >= toDepth; --boundary)
        {
            total += AsmOnDuration(boundary);
        }
    }
    return total;
}

double
MmWaveEnbNetDevice::AsmTxPowerW() const
{
    if (m_ccMap.empty())
    {
        return 1.0;
    }
    Ptr<MmWaveEnbPhy> phy = GetPhy();
    if (!phy)
    {
        return 1.0;
    }
    return std::pow(10.0, (phy->GetTxPower() - 30.0) / 10.0);
}

double
MmWaveEnbNetDevice::AsmStablePowerW(uint8_t mode) const
{
    switch (mode)
    {
    case 1:
        return AsmTxPowerW() + m_rfPower + m_bbPower + m_controlPower;
    case 2:
        return m_rfPower + m_bbPower + m_controlPower;
    case 3:
        return m_bbPower + m_controlPower;
    case 4:
        return m_controlPower;
    case 5:
        return m_minimalPower;
    default:
        return 0.0;
    }
}

double
MmWaveEnbNetDevice::AsmOffIntervalPowerW(int fromDepth, int boundary) const
{
    double power = 0.0;
    // The PA is the component turning off on boundary 0, so radiated power is
    // already gone. fromDepth selects the same off sequence.
    (void)fromDepth;
    if (boundary < 1)
    {
        power += m_rfPower;
    }
    if (boundary < 2)
    {
        power += m_bbPower;
    }
    if (boundary < 3)
    {
        power += m_controlPower;
    }
    return power;
}

double
MmWaveEnbNetDevice::AsmOnIntervalPowerW(int boundary) const
{
    switch (boundary)
    {
    case 0:
        return m_paWakePower + m_rfPower + m_bbPower + m_controlPower;
    case 1:
        return m_rfWakePower + m_bbPower + m_controlPower;
    case 2:
        return m_bbWakePower + m_controlPower;
    case 3:
        return m_controlWakePower;
    case 4:
        return m_minimalWakePower;
    default:
        return 0.0;
    }
}

double
MmWaveEnbNetDevice::AsmPowerW() const
{
    if (!m_asmStateMachine || m_asmPhase == 0)
    {
        return AsmStablePowerW(m_asmStateMachine ? m_asmCurrent : 1);
    }
    Time now = Simulator::Now();
    int fromDepth = static_cast<int>(m_asmCurrent) - 1;
    int toDepth = static_cast<int>(m_asmTarget) - 1;
    if (now >= m_asmTransEnd)
    {
        return AsmStablePowerW(m_asmTarget);
    }
    Time elapsed = now - m_asmTransStart;
    if (elapsed.IsNegative())
    {
        elapsed = Seconds(0);
    }
    Time cursor = Seconds(0);
    if (toDepth > fromDepth)
    {
        for (int boundary = fromDepth; boundary < toDepth; ++boundary)
        {
            Time len = AsmOffDuration(boundary);
            if (elapsed < cursor + len)
            {
                return AsmOffIntervalPowerW(fromDepth, boundary);
            }
            cursor += len;
        }
    }
    else
    {
        for (int boundary = fromDepth - 1; boundary >= toDepth; --boundary)
        {
            Time len = AsmOnDuration(boundary);
            if (elapsed < cursor + len)
            {
                return AsmOnIntervalPowerW(boundary);
            }
            cursor += len;
        }
    }
    return AsmStablePowerW(m_asmTarget);
}

void
MmWaveEnbNetDevice::RecvGymControlReady(Ptr<MmWaveEnbNetDevice> netDev, std::string /*context*/)
{
    if (!netDev)
    {
        return;
    }
    netDev->ApplyAsmControlFile();
}

void
MmWaveEnbNetDevice::ApplyAsmToSchedulers()
{
    NS_LOG_FUNCTION(this << +m_asmMode);
    for (auto& it : m_ccMap)
    {
        Ptr<MmWaveComponentCarrierEnb> cc = DynamicCast<MmWaveComponentCarrierEnb>(it.second);
        if (cc && cc->GetMacScheduler())
        {
            cc->GetMacScheduler()->SetAsmMode(m_asmMode);
        }
    }
}

Ptr<E2Termination>
MmWaveEnbNetDevice::GetE2Termination() const
{
    return m_e2term;
}

void
MmWaveEnbNetDevice::SetE2Termination(Ptr<E2Termination> e2term)
{
    m_e2term = e2term;

    NS_LOG_DEBUG("Register E2SM");

    if (m_e2term)
    {
        Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription>(1);
        e2term->RegisterKpmCallbackToE2Sm(
            2,
            kpmFd,
            std::bind(&MmWaveEnbNetDevice::KpmSubscriptionCallback, this, std::placeholders::_1));

        Ptr<RicControlFunctionDescription> ricCtrlFd = Create<RicControlFunctionDescription>();
        e2term->RegisterSmCallbackToE2Sm(
            3,
            ricCtrlFd,
            std::bind(&MmWaveEnbNetDevice::ControlMessageReceivedCallback,
                      this,
                      std::placeholders::_1));
    }
}

void
MmWaveEnbNetDevice::ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_DEBUG("MmWaveEnbNetDevice::ControlMessageReceivedCallback");
    NS_LOG_DEBUG("PDU present " << (sub_req_pdu ? sub_req_pdu->present : -1));
}

std::string
MmWaveEnbNetDevice::GetImsiString(uint64_t imsi)
{
    std::string ueImsi = std::to_string(imsi);
    std::string ueImsiComplete{};
    if (ueImsi.length() == 1)
    {
        ueImsiComplete = "0000" + ueImsi;
    }
    else if (ueImsi.length() == 2)
    {
        ueImsiComplete = "000" + ueImsi;
    }
    else
    {
        ueImsiComplete = "00" + ueImsi;
    }
    return ueImsiComplete;
}

Ptr<KpmIndicationHeader>
MmWaveEnbNetDevice::BuildRicIndicationHeader(std::string plmId,
                                             std::string gnbId,
                                             uint16_t nrCellId)
{
    if (m_e2term)
    {
        KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
        headerValues.m_plmId = plmId;
        headerValues.m_gnbId = gnbId;
        headerValues.m_nrCellId = nrCellId;
        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();
        NS_LOG_DEBUG("NR plmid " << plmId << " gnbId " << gnbId << " nrCellId " << nrCellId);
        NS_LOG_DEBUG("Timestamp " << timestamp);
        headerValues.m_timestamp = timestamp;

        Ptr<KpmIndicationHeader> header =
            Create<KpmIndicationHeader>(KpmIndicationHeader::GlobalE2nodeType::gNB, headerValues);

        return header;
    }
    else
    {
        return nullptr;
    }
}

Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageCuUp(std::string plmId)
{
    (void)plmId;
    // get <rnti, UeManager> map of connected UEs
    auto ueMap = m_rrc->GetUeMap();
    // gNB-wide PDCP volume in downlink
    double cellDlTxVolume = 0;
    // rx bytes in downlink
    double cellDlRxVolume = 0;

    // sum of the per-user average latency
    double perUserAverageLatencySum = 0;

    std::unordered_map<uint64_t, std::string> uePmString{};

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        std::string ueImsiComplete = GetImsiString(imsi);

        // double rxDlPackets = m_e2PdcpStatsCalculator->GetDlRxPackets(imsi, 3); // LCID 3 is used
        // for data
        uint64_t txDlPackets =
            m_e2PdcpStatsCalculator->GetDlTxPackets(imsi, 3); // LCID 3 is used for data
        uint64_t txBytes = std::round (
            m_e2PdcpStatsCalculator->GetDlTxData(imsi, 3) * 8 / 1e3); // in kbit, not byte
        uint64_t rxBytes =
            m_e2PdcpStatsCalculator->GetDlRxData(imsi, 3) * 8 / 1e3; // in kbit, not byte
        cellDlTxVolume += txBytes;
        cellDlRxVolume += rxBytes;

        uint64_t txPdcpPduNrRlc = 0;
        uint64_t txPdcpPduBytesNrRlc = 0;


        auto rlcMap = ue.second->GetRlcMap(); // secondary-connected RLCs
        NS_LOG_INFO("About to get into the RLC map for ue " << std::to_string(imsi) << " size " << rlcMap.size());

        for (auto drb : rlcMap)
        {
            txPdcpPduNrRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
            txPdcpPduBytesNrRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
            NS_LOG_INFO("ue id " << std::to_string(imsi) << " txPdcpPduNrRlc " << std::to_string(txPdcpPduNrRlc));
            NS_LOG_INFO("ue id " << std::to_string(imsi) << " txPdcpPduBytesNrRlc " << std::to_string(txPdcpPduBytesNrRlc));
            drb.second->m_rlc->ResetRlcCounters();
        }
        txPdcpPduBytesNrRlc = std::round (txPdcpPduBytesNrRlc * 8 / 1e3); // in kbit, not byte

        NS_LOG_INFO("txDlPackets (" << txDlPackets << ") vs txPdcpPduNrRlc (" << txPdcpPduNrRlc
                                    << ") and txBytes (" << txBytes << ") vs txPdcpPduBytesNrRlc ("
                                    << txPdcpPduBytesNrRlc << ")\n");

        /*
         * There is a corner case after a handover where the reported NR RLC packets and bytes
         * exceed the reported cell values from the PDCP. This occurs because RLC buffers are
         * transferred between cells during the handover, and if this happens during the reset
         * of an E2 indication periodicity, mismatches can arise.
         *
         * For now, we calculate the NR values from the lte enb net device. Here below some
         * possible solutions on how we address this by ignoring the issue and removing the
         * additional packet on the RLC. However, a more comprehensive fix could involve one 
         * of the following approaches:
         *
         * 1. Drop traffic at or before the PDCP until the UE reconfiguration is complete, and
         *    consequently avoid logging these packets.
         * 2. Move the trace setup to when the bearer (i.e., RLC and PDCP objects) is created,
         *    and delete the trace if the UE reconfiguration is unsuccessful.
         * 3. As an alternative (not a fix) for cases where RRC procedures are not of concern,
         *    use `RrcIdeal`.
         */

        if (txDlPackets < txPdcpPduNrRlc)
        {
        
            NS_LOG_WARN("ue id " << std::to_string(imsi) << " txDlPackets (" << txDlPackets
                                      << ") < txPdcpPduNrRlc (" << txPdcpPduNrRlc
                                      << ") on mmWaveCell");

            // // Corner case: just one packet, we remove it and move forward
            // if (txDlPackets + 1 == txPdcpPduNrRlc)
            // {
            //     NS_LOG_WARN("ue id " << std::to_string(imsi) << " txDlPackets (" << txDlPackets
            //                           << ") + 1 == txPdcpPduNrRlc (" << txPdcpPduNrRlc
            //                           << "), removing one packet");
            //     txPdcpPduNrRlc = txDlPackets;
            //     txBytes = txPdcpPduBytesNrRlc;
            // }
            // else
            // {
            //     // Something else is wrong and we need to investigate, launch assert
            //     NS_ASSERT_MSG(txDlPackets >= txPdcpPduNrRlc && txBytes >= txPdcpPduBytesNrRlc,
            //                   "txDlPackets (" << txDlPackets << ") < txPdcpPduNrRlc ("
            //                                   << txPdcpPduNrRlc << ") txBytes (" << txBytes
            //                                   << ") < txPdcpPduBytesNrRlc (" << txPdcpPduBytesNrRlc
            //                                   << ")");
            // }
        }

        NS_LOG_INFO("ue id " << std::to_string(imsi) << " final txPdcpPduBytesNrRlc after *= 8 / 1e3: " << std::to_string(txPdcpPduBytesNrRlc));
        double pdcpLatency = m_e2PdcpStatsCalculator->GetDlDelay(imsi, 3) / 1e5; // unit: x 0.1 ms
        perUserAverageLatencySum += pdcpLatency;

        double pdcpThroughput = txBytes / m_e2Periodicity;   // unit kbps
        double pdcpThroughputRx = rxBytes / m_e2Periodicity; // unit kbps

        if (m_drbThrDlPdcpBasedComputationUeid.find(imsi) !=
            m_drbThrDlPdcpBasedComputationUeid.end())
        {
            m_drbThrDlPdcpBasedComputationUeid.at(imsi) += pdcpThroughputRx;
        }
        else
        {
            m_drbThrDlPdcpBasedComputationUeid[imsi] = pdcpThroughputRx;
        }

        // compute bitrate based on RLC statistics, decoupled from pdcp throughput
        double rlcLatencyS = m_e2RlcStatsCalculator->GetDlDelay(imsi, 3);
        double rlcLatency = rlcLatencyS / 1e9; // unit: s
        NS_LOG_INFO("rlcLatencyS (" << rlcLatencyS << ") rlcLatency (" << rlcLatency << ")");

        double pduSizeBytes = m_e2RlcStatsCalculator->GetDlPduSizeStats(imsi, 3)[0];
        double pduSize = pduSizeBytes * 8.0 / 1e3; // unit kbit
        double rlcBitrate = (rlcLatency == 0) ? 0 : pduSize / rlcLatency;     // unit kbit/s
        
        NS_LOG_INFO("pduSizeBytes (" << pduSizeBytes << ") pduSize (" << pduSize << ") rlcBitrate (" << rlcBitrate << ")");

        m_drbThrDlUeid[imsi] = rlcBitrate;

        if (m_e2term)
        {
            E2UeKpm& ueKpm = m_e2UeKpm[imsi];
            ueKpm.imsiComplete = ueImsiComplete;
            ueKpm.txPdcpPduBytesNrRlc = txPdcpPduBytesNrRlc;
            ueKpm.txPdcpPduNrRlc = txPdcpPduNrRlc;
            ueKpm.drbThrDlUeid = rlcBitrate;
        }

        NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                     << " " << m_cellId << " cell, connected UE with IMSI " << imsi
                     << " ueImsiString " << ueImsiComplete << " txDlPackets " << txDlPackets
                     << " txDlPacketsNr " << txPdcpPduNrRlc << " txBytes " << txBytes << " rxBytes "
                     << rxBytes << " txDlBytesNr " << txPdcpPduBytesNrRlc << " pdcpLatency "
                     << pdcpLatency << " pdcpThroughput " << pdcpThroughput << " rlcBitrate "
                     << rlcBitrate);

        m_e2PdcpStatsCalculator->ResetResultsForImsiLcid(imsi, 3);

        // TODO enable this back once the reports are fixed
        // uePmString.insert(std::make_pair(imsi,
        //                                  ",,,,,," + std::to_string(txPdcpPduBytesNrRlc) + "," +
        //                                      std::to_string(txPdcpPduNrRlc)));
        uePmString.insert(std::make_pair(imsi,
                                         ",,,,,,,,,"));
    }

    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                 << " " << m_cellId << " cell volume mmWave " << cellDlTxVolume);

    if (m_enableGymFileLogging && m_sendCuUp)
    {
        std::ofstream csv{};
        csv.open(m_cuUpFileName.c_str(), std::ios_base::app);
        if (!csv.is_open())
        {
            NS_FATAL_ERROR("Can't open file " << m_cuUpFileName.c_str());
        }

        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();

        // the string is timestamp, ueImsiComplete, DRB.PdcpSduDelayDl(cellAverageLatency),
        // m_pDCPBytesUL(0),m_pDCPBytesDL(cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID
        // (txBytes),Tot.PdcpSduNbrDl.UEID (txDlPackets),
        // DRB.PdcpSduBitRateDl.UEID(pdcpThroughput),DRB.PdcpSduDelayDl.UEID(pdcpLatency),
        // txPdcpPduLteRlc,txPdcpPduBytesLteRlc,
        // QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
        // DRB.PdcpPduNbrDl.Qos.UEID(txPdcpPduNrRlc)

        for (auto ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string ueImsiComplete = GetImsiString(imsi);

            auto uePms = uePmString.find(imsi)->second;

            std::string to_print = std::to_string(timestamp) + "," + ueImsiComplete + "," + "," +
                                   "," + "," + uePms + "\n";

            csv << to_print;
        }
        csv.close();
    }
    return nullptr;
}

template <typename A, typename B>
std::pair<B, A>
flip_pair(const std::pair<A, B>& p)
{
    return std::pair<B, A>(p.second, p.first);
}

template <typename A, typename B>
std::multimap<B, A>
flip_map(const std::map<A, B>& src)
{
    std::multimap<B, A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), flip_pair<A, B>);
    return dst;
}

void
MmWaveEnbNetDevice::SaveActiveUes(void)
{
    NS_LOG_FUNCTION(this);
    m_ueRrcMean.push_back(m_rrc->GetUeMap().size());

    // We register this every 10 ms
    Simulator::Schedule(MilliSeconds(10), &MmWaveEnbNetDevice::SaveActiveUes, this);
}

long
MmWaveEnbNetDevice::ComputeMeanUes(void)
{
    NS_LOG_FUNCTION(this);
    if (m_ueRrcMean.size() == 0)
    {
        return 0;
    }
    // result is an integer according to the standard
    long meanUes =
        std::round(std::accumulate(m_ueRrcMean.begin(), m_ueRrcMean.end(), 0) / m_ueRrcMean.size());

    m_ueRrcMean.clear();
    return meanUes;
}

Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageCuCp(std::string plmId)
{
    (void)plmId;
    auto ueMap = m_rrc->GetUeMap();
    long meanRrcUes = ComputeMeanUes();

    std::unordered_map<uint64_t, std::string> uePmString{};

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        std::string ueImsiComplete = GetImsiString(imsi);

        // This shall be created in connected mode and sent through the E2 Interface
        // Since now they are now integrated in the asn1 definiton and they are leaking
        // I'll just comment them
        // Ptr<MeasurementItemList> ueVal = Create<MeasurementItemList> (ueImsiComplete);

        long numDrb = ue.second->GetDrbMap().size();

        // if (!m_reducedPmValues && !indicationMessageHelper->IsOffline ())
        //   {
        //     ueVal->AddItem<long> ("DRB.EstabSucc.5QI.UEID", numDrb);
        //     ueVal->AddItem<long> ("DRB.RelActNbr.5QI.UEID", 0); // not modeled in the simulator
        //   }

        // create L3 RRC reports

        // for the same cell
        double sinrThisCell = 10 * std::log10(m_l3sinrMap[imsi][m_cellId]);
        double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr(sinrThisCell);

        if (m_e2term)
        {
            E2UeKpm& ueKpm = m_e2UeKpm[imsi];
            ueKpm.imsiComplete = ueImsiComplete;
            ueKpm.numDrb = numDrb;
            ueKpm.sinrServ = sinrThisCell;
            ueKpm.sinrServ3gpp = convertedSinr;
            ueKpm.servCell = m_cellId;
        }

        // NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        //              << " enbdev " << m_cellId << " UE " << imsi << " L3 serving SINR "
        //              << sinrThisCell << " L3 serving SINR 3gpp " << convertedSinr);

        std::string servingStr = std::to_string(numDrb) + "," + std::to_string(0) + "," +
                                 std::to_string(m_cellId) + "," + std::to_string(imsi) + "," +
                                 std::to_string(sinrThisCell) + "," + std::to_string(convertedSinr);

        // For the neighbors
        // TODO create double map, imsi -> cell -> sinr
        // TODO store at most 8 reports for each UE, as per the standard

        double sinr;
        std::string neighStr;

        // invert key and value in sortFlipMap, then sort by value
        std::multimap<long double, uint16_t> sortFlipMap = flip_map(m_l3sinrMap[imsi]);
        // new sortFlipMap structure sortFlipMap < sinr, cellId >
        // The assumption is that the first cell in the scenario is always LTE and the rest NR
        uint16_t nNeighbours = E2SM_REPORT_MAX_NEIGH;
        if (m_l3sinrMap[imsi].size() < nNeighbours)
        {
            nNeighbours = m_l3sinrMap[imsi].size() - 1;
        }
        int itIndex = 0;
        // Save only the first E2SM_REPORT_MAX_NEIGH SINR for each UE which represent the best
        // values among all the SINRs detected by all the cells
        for (std::map<long double, uint16_t>::iterator it = --sortFlipMap.end();
             it != --sortFlipMap.begin() && itIndex < nNeighbours;
             it--)
        {
            uint16_t cellId = it->second;
            if (cellId != m_cellId)
            {
                sinr = 10 * std::log10(it->first); // now SINR is a key due to the sort of the map
                convertedSinr = L3RrcMeasurements::ThreeGppMapSinr(sinr);
                if (m_e2term && itIndex < E2SM_REPORT_MAX_NEIGH)
                {
                    E2UeKpm& ueKpm = m_e2UeKpm[imsi];
                    ueKpm.neighId[itIndex] = cellId;
                    ueKpm.neighSinr[itIndex] = sinr;
                    ueKpm.neighSinr3gpp[itIndex] = convertedSinr;
                }
                NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                             << " enbdev " << m_cellId << " UE " << imsi << " L3 neigh " << cellId
                             << " SINR " << sinr << " sinr encoded " << convertedSinr
                             << " first insert");
                neighStr += "," + std::to_string(cellId) + "," + std::to_string(sinr) + "," +
                            std::to_string(convertedSinr);
                itIndex++;
            }
        }
        for (int i = nNeighbours; i < E2SM_REPORT_MAX_NEIGH; i++)
        {
            neighStr += ",,,";
        }

        uePmString.insert(std::make_pair(imsi, servingStr + neighStr));
    }

    if (m_enableGymFileLogging && m_sendCuCp)
    {
        std::ofstream csv{};
        csv.open(m_cuCpFileName.c_str(), std::ios_base::app);
        if (!csv.is_open())
        {
            NS_FATAL_ERROR("Can't open file " << m_cuCpFileName.c_str());
        }

        NS_LOG_DEBUG("m_cuCpFileName open " << m_cuCpFileName);

        // the string is timestamp, ueImsiComplete, numActiveUes,RRC.ConnMean,
        // DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0), L3 serving Id (m_cellId), UE
        // (imsi), L3 serving SINR, L3 serving SINR 3gpp, L3 neigh Id (cellId), L3 neigh Sinr, L3
        // neigh SINR 3gpp (convertedSinr) The values for L3 neighbour cells are repeated for each
        // neighbour (7 times in this implementation)

        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();

        for (auto ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string ueImsiComplete = GetImsiString(imsi);

            auto uePms = uePmString.find(imsi)->second;

            std::string to_print = std::to_string(timestamp) + "," + ueImsiComplete + "," +
                                   std::to_string(ueMap.size()) + "," + std::to_string(meanRrcUes) +
                                   "," + uePms + "\n";

            // NS_LOG_DEBUG(to_print);

            csv << to_print;
        }
        csv.close();
    }
    return nullptr;
}

uint32_t
MmWaveEnbNetDevice::GetRlcBufferOccupancy(Ptr<LteRlc> rlc) const
{
    if (DynamicCast<LteRlcAm>(rlc))
    {
        return DynamicCast<LteRlcAm>(rlc)->GetTxBufferSize();
    }
    else if (DynamicCast<LteRlcUm>(rlc))
    {
        return DynamicCast<LteRlcUm>(rlc)->GetTxBufferSize();
    }
    else if (DynamicCast<LteRlcUmLowLat>(rlc))
    {
        return DynamicCast<LteRlcUmLowLat>(rlc)->GetTxBufferSize();
    }
    else
    {
        return 0;
    }
}

Ptr<KpmIndicationMessage>
MmWaveEnbNetDevice::BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId)
{
    auto ueMap = m_rrc->GetUeMap();

    uint32_t macPduCellSpecific = 0;
    uint32_t macPduInitialCellSpecific = 0;
    uint32_t macVolumeCellSpecific = 0;
    uint32_t macQpskCellSpecific = 0;
    uint32_t mac16QamCellSpecific = 0;
    uint32_t mac64QamCellSpecific = 0;
    uint32_t macRetxCellSpecific = 0;
    uint32_t macMac04CellSpecific = 0;
    uint32_t macMac59CellSpecific = 0;
    uint32_t macMac1014CellSpecific = 0;
    uint32_t macMac1519CellSpecific = 0;
    uint32_t macMac2024CellSpecific = 0;
    uint32_t macMac2529CellSpecific = 0;

    uint32_t macSinrBin1CellSpecific = 0;
    uint32_t macSinrBin2CellSpecific = 0;
    uint32_t macSinrBin3CellSpecific = 0;
    uint32_t macSinrBin4CellSpecific = 0;
    uint32_t macSinrBin5CellSpecific = 0;
    uint32_t macSinrBin6CellSpecific = 0;
    uint32_t macSinrBin7CellSpecific = 0;

    uint32_t rlcBufferOccupCellSpecific = 0;

    uint32_t macPrbsCellSpecific = 0;

    // [nsoran-qos] 139 was 100 MHz + numerology 2 (60 kHz SCS). Use actual RB count.
    Ptr<MmWavePhyMacCommon> phyMacCfg = GetMac() ? GetMac()->GetConfigurationParameters() : nullptr;
    long dlAvailablePrbs =
        (phyMacCfg && phyMacCfg->GetNumRb() > 0) ? static_cast<long>(phyMacCfg->GetNumRb()) : 139;
    long ulAvailablePrbs = dlAvailablePrbs;

    std::unordered_map<uint64_t, std::string> uePmStringDu{};

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        std::string ueImsiComplete = GetImsiString(imsi);
        uint16_t rnti = ue.second->GetRnti();

        uint32_t macPduUe = m_e2DuCalculator->GetMacPduUeSpecific(rnti, m_cellId);

        macPduCellSpecific += macPduUe;
        m_macPduCellSpecific = macPduCellSpecific;

        uint32_t macPduInitialUe =
            m_e2DuCalculator->GetMacPduInitialTransmissionUeSpecific(rnti, m_cellId);
        macPduInitialCellSpecific += macPduInitialUe;

        uint32_t macVolume = m_e2DuCalculator->GetMacVolumeUeSpecific(rnti, m_cellId);
        macVolumeCellSpecific += macVolume;
        m_macVolumeCellSpecific = macVolumeCellSpecific;

        uint32_t macQpsk = m_e2DuCalculator->GetMacPduQpskUeSpecific(rnti, m_cellId);
        macQpskCellSpecific += macQpsk;

        uint32_t mac16Qam = m_e2DuCalculator->GetMacPdu16QamUeSpecific(rnti, m_cellId);
        mac16QamCellSpecific += mac16Qam;

        uint32_t mac64Qam = m_e2DuCalculator->GetMacPdu64QamUeSpecific(rnti, m_cellId);
        mac64QamCellSpecific += mac64Qam;

        uint32_t macRetx = m_e2DuCalculator->GetMacPduRetransmissionUeSpecific(rnti, m_cellId);
        macRetxCellSpecific += macRetx;

        // Numerator = (Sum of number of symbols across all rows (TTIs) group by cell ID and UE ID
        // within a given time window)
        double macNumberOfSymbols =
            m_e2DuCalculator->GetMacNumberOfSymbolsUeSpecific(rnti, m_cellId);

        auto phyMac = GetMac()->GetConfigurationParameters();
        // Denominator = (Periodicity of the report time window in ms*number of TTIs per ms*14)
        Time reportingWindow =
            Simulator::Now() - m_e2DuCalculator->GetLastResetTime(rnti, m_cellId);
        double denominatorPrb =
            std::ceil(reportingWindow.GetNanoSeconds() / phyMac->GetSlotPeriod().GetNanoSeconds()) *
            14;

        NS_LOG_DEBUG("macNumberOfSymbols " << macNumberOfSymbols << " denominatorPrb "
                                           << denominatorPrb);

        // Average Number of PRBs allocated for the UE = (NR/DR)*N_RB
        double macPrb = 0;
        if (denominatorPrb != 0)
        {
            // [nsoran-qos] was hardcoded 139 (100 MHz, numerology 2)
            macPrb = macNumberOfSymbols / denominatorPrb * static_cast<double>(dlAvailablePrbs);
        }
        macPrbsCellSpecific += macPrb;

        uint32_t macMac04 = m_e2DuCalculator->GetMacMcs04UeSpecific(rnti, m_cellId);
        macMac04CellSpecific += macMac04;

        uint32_t macMac59 = m_e2DuCalculator->GetMacMcs59UeSpecific(rnti, m_cellId);
        macMac59CellSpecific += macMac59;

        uint32_t macMac1014 = m_e2DuCalculator->GetMacMcs1014UeSpecific(rnti, m_cellId);
        macMac1014CellSpecific += macMac1014;

        uint32_t macMac1519 = m_e2DuCalculator->GetMacMcs1519UeSpecific(rnti, m_cellId);
        macMac1519CellSpecific += macMac1519;

        uint32_t macMac2024 = m_e2DuCalculator->GetMacMcs2024UeSpecific(rnti, m_cellId);
        macMac2024CellSpecific += macMac2024;

        uint32_t macMac2529 = m_e2DuCalculator->GetMacMcs2529UeSpecific(rnti, m_cellId);
        macMac2529CellSpecific += macMac2529;

        uint32_t macSinrBin1 = m_e2DuCalculator->GetMacSinrBin1UeSpecific(rnti, m_cellId);
        macSinrBin1CellSpecific += macSinrBin1;

        uint32_t macSinrBin2 = m_e2DuCalculator->GetMacSinrBin2UeSpecific(rnti, m_cellId);
        macSinrBin2CellSpecific += macSinrBin2;

        uint32_t macSinrBin3 = m_e2DuCalculator->GetMacSinrBin3UeSpecific(rnti, m_cellId);
        macSinrBin3CellSpecific += macSinrBin3;

        uint32_t macSinrBin4 = m_e2DuCalculator->GetMacSinrBin4UeSpecific(rnti, m_cellId);
        macSinrBin4CellSpecific += macSinrBin4;

        uint32_t macSinrBin5 = m_e2DuCalculator->GetMacSinrBin5UeSpecific(rnti, m_cellId);
        macSinrBin5CellSpecific += macSinrBin5;

        uint32_t macSinrBin6 = m_e2DuCalculator->GetMacSinrBin6UeSpecific(rnti, m_cellId);
        macSinrBin6CellSpecific += macSinrBin6;

        uint32_t macSinrBin7 = m_e2DuCalculator->GetMacSinrBin7UeSpecific(rnti, m_cellId);
        macSinrBin7CellSpecific += macSinrBin7;

        // get buffer occupancy info
        uint32_t rlcBufferOccup = 0;
        auto drbMap = ue.second->GetDrbMap();
        for (auto drb : drbMap)
        {
            auto rlc = drb.second->m_rlc;
            rlcBufferOccup += GetRlcBufferOccupancy(rlc);
        }
        auto rlcMap = ue.second->GetRlcMap(); // secondary-connected RLCs
        for (auto drb : rlcMap)
        {
            auto rlc = drb.second->m_rlc;
            rlcBufferOccup += GetRlcBufferOccupancy(rlc);
        }
        rlcBufferOccupCellSpecific += rlcBufferOccup;

        NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                     << " " << m_cellId << " cell, connected UE with IMSI " << imsi << " rnti "
                     << rnti << " macPduUe " << macPduUe << " macPduInitialUe " << macPduInitialUe
                     << " macVolume " << macVolume << " macQpsk " << macQpsk << " mac16Qam "
                     << mac16Qam << " mac64Qam " << mac64Qam << " macRetx " << macRetx << " macPrb "
                     << macPrb << " macMac04 " << macMac04 << " macMac59 " << macMac59
                     << " macMac1014 " << macMac1014 << " macMac1519 " << macMac1519
                     << " macMac2024 " << macMac2024 << " macMac2529 " << macMac2529
                     << " macSinrBin1 " << macSinrBin1 << " macSinrBin2 " << macSinrBin2
                     << " macSinrBin3 " << macSinrBin3 << " macSinrBin4 " << macSinrBin4
                     << " macSinrBin5 " << macSinrBin5 << " macSinrBin6 " << macSinrBin6
                     << " macSinrBin7 " << macSinrBin7 << " rlcBufferOccup " << rlcBufferOccup);

        // UE-specific Downlink IP combined EN-DC throughput from LTE eNB. Unit is kbps. Pdcp based
        // computation This value is not requested anymore, so it has been removed from the
        // delivery, but it will be still logged;
        double drbThrDlPdcpBasedUeid = m_drbThrDlPdcpBasedComputationUeid.find(imsi) !=
                                               m_drbThrDlPdcpBasedComputationUeid.end()
                                           ? m_drbThrDlPdcpBasedComputationUeid.at(imsi)
                                           : 0;

        // UE-specific Downlink IP combined EN-DC throughput from LTE eNB. Unit is kbps. Rlc based
        // computation
        double drbThrDlUeid =
            m_drbThrDlUeid.find(imsi) != m_drbThrDlUeid.end() ? m_drbThrDlUeid.at(imsi) : 0;

        if (m_e2term)
        {
            E2UeKpm& ueKpm = m_e2UeKpm[imsi];
            ueKpm.imsiComplete = ueImsiComplete;
            ueKpm.macPduUe = macPduUe;
            ueKpm.macPduInitialUe = macPduInitialUe;
            ueKpm.macQpsk = macQpsk;
            ueKpm.mac16Qam = mac16Qam;
            ueKpm.mac64Qam = mac64Qam;
            ueKpm.macRetx = macRetx;
            ueKpm.macVolume = macVolume;
            ueKpm.macPrb = macPrb;
            ueKpm.macMac04 = macMac04;
            ueKpm.macMac59 = macMac59;
            ueKpm.macMac1014 = macMac1014;
            ueKpm.macMac1519 = macMac1519;
            ueKpm.macMac2024 = macMac2024;
            ueKpm.macMac2529 = macMac2529;
            ueKpm.macSinrBin1 = macSinrBin1;
            ueKpm.macSinrBin2 = macSinrBin2;
            ueKpm.macSinrBin3 = macSinrBin3;
            ueKpm.macSinrBin4 = macSinrBin4;
            ueKpm.macSinrBin5 = macSinrBin5;
            ueKpm.macSinrBin6 = macSinrBin6;
            ueKpm.macSinrBin7 = macSinrBin7;
            ueKpm.rlcBufferOccup = rlcBufferOccup;
            ueKpm.drbThrDlUeid = drbThrDlUeid;
        }

        uePmStringDu.insert(std::make_pair(
            imsi,
            std::to_string(macPduUe) + "," + std::to_string(macPduInitialUe) + "," +
                std::to_string(macQpsk) + "," + std::to_string(mac16Qam) + "," +
                std::to_string(mac64Qam) + "," + std::to_string(macRetx) + "," +
                std::to_string(macVolume) + "," + std::to_string(macPrb) + "," +
                std::to_string(macMac04) + "," + std::to_string(macMac59) + "," +
                std::to_string(macMac1014) + "," + std::to_string(macMac1519) + "," +
                std::to_string(macMac2024) + "," + std::to_string(macMac2529) + "," +
                std::to_string(macSinrBin1) + "," + std::to_string(macSinrBin2) + "," +
                std::to_string(macSinrBin3) + "," + std::to_string(macSinrBin4) + "," +
                std::to_string(macSinrBin5) + "," + std::to_string(macSinrBin6) + "," +
                std::to_string(macSinrBin7) + "," + std::to_string(rlcBufferOccup) + ',' +
                std::to_string(drbThrDlUeid) + ',' + std::to_string(drbThrDlPdcpBasedUeid)));

        // reset UE
        m_e2DuCalculator->ResetPhyTracesForRntiCellId(rnti, m_cellId);
    }

    m_drbThrDlPdcpBasedComputationUeid.clear();
    m_drbThrDlUeid.clear();

    // Denominator = (Total number of rows (TTIs) within a given time window* 14)
    // Numerator = (Sum of number of symbols across all rows (TTIs) group by cell ID within a given
    // time window) * N_RB
    double prbUtilizationDl = macPrbsCellSpecific;
    // double m_prbUtilizationDlAttr = prbUtilizationDl;

    NS_LOG_DEBUG(
        Simulator::Now().GetSeconds()
        << " " << m_cellId << " cell, connected UEs number " << ueMap.size()
        << " macPduCellSpecific " << macPduCellSpecific << " macPduInitialCellSpecific "
        << macPduInitialCellSpecific << " macVolumeCellSpecific " << macVolumeCellSpecific
        << " macQpskCellSpecific " << macQpskCellSpecific << " mac16QamCellSpecific "
        << mac16QamCellSpecific << " mac64QamCellSpecific " << mac64QamCellSpecific
        << " macRetxCellSpecific " << macRetxCellSpecific << " macPrbsCellSpecific "
        << macPrbsCellSpecific //<< " " << macNumberOfSymbolsCellSpecific << " " << denominatorPrb
        << " macMac04CellSpecific " << macMac04CellSpecific << " macMac59CellSpecific "
        << macMac59CellSpecific << " macMac1014CellSpecific " << macMac1014CellSpecific
        << " macMac1519CellSpecific " << macMac1519CellSpecific << " macMac2024CellSpecific "
        << macMac2024CellSpecific << " macMac2529CellSpecific " << macMac2529CellSpecific
        << " macSinrBin1CellSpecific " << macSinrBin1CellSpecific << " macSinrBin2CellSpecific "
        << macSinrBin2CellSpecific << " macSinrBin3CellSpecific " << macSinrBin3CellSpecific
        << " macSinrBin4CellSpecific " << macSinrBin4CellSpecific << " macSinrBin5CellSpecific "
        << macSinrBin5CellSpecific << " macSinrBin6CellSpecific " << macSinrBin6CellSpecific
        << " macSinrBin7CellSpecific " << macSinrBin7CellSpecific);

    long qci = 1;
    long dlPrbUsage = std::min((long)(prbUtilizationDl / dlAvailablePrbs * 100),
                               (long)100); // percentage of used PRBs
    long ulPrbUsage = 0;                   // TODO for future implementation

    double asmPowerW = 0.0;
    double asmEnergyJ = 0.0;
    if ((m_enableGymFileLogging && m_sendDu) || m_e2term)
    {
        asmPowerW = ConsumeAveragePowerW(m_e2Periodicity);
        asmEnergyJ = asmPowerW * m_e2Periodicity;
    }
    if (m_e2term)
    {
        m_e2CellValid = true;
        m_e2CellId = nrCellId;
        m_e2NumActiveUes = static_cast<uint16_t>(ueMap.size());
        m_e2MacPdu = macPduCellSpecific;
        m_e2MacPduInitial = macPduInitialCellSpecific;
        m_e2MacQpsk = macQpskCellSpecific;
        m_e2Mac16Qam = mac16QamCellSpecific;
        m_e2Mac64Qam = mac64QamCellSpecific;
        m_e2PrbUtilizationDl = prbUtilizationDl;
        m_e2MacRetx = macRetxCellSpecific;
        m_e2MacVolume = macVolumeCellSpecific;
        m_e2MacMac04 = macMac04CellSpecific;
        m_e2MacMac59 = macMac59CellSpecific;
        m_e2MacMac1014 = macMac1014CellSpecific;
        m_e2MacMac1519 = macMac1519CellSpecific;
        m_e2MacMac2024 = macMac2024CellSpecific;
        m_e2MacMac2529 = macMac2529CellSpecific;
        m_e2SinrBin1 = macSinrBin1CellSpecific;
        m_e2SinrBin2 = macSinrBin2CellSpecific;
        m_e2SinrBin3 = macSinrBin3CellSpecific;
        m_e2SinrBin4 = macSinrBin4CellSpecific;
        m_e2SinrBin5 = macSinrBin5CellSpecific;
        m_e2SinrBin6 = macSinrBin6CellSpecific;
        m_e2SinrBin7 = macSinrBin7CellSpecific;
        m_e2RlcBuffer = rlcBufferOccupCellSpecific;
        m_e2ActiveUeDl = ueMap.size();
        m_e2DlAvailablePrbs = dlAvailablePrbs;
        m_e2UlAvailablePrbs = ulAvailablePrbs;
        m_e2Qci = qci;
        m_e2DlPrbUsage = dlPrbUsage;
        m_e2UlPrbUsage = ulPrbUsage;
        m_e2AsmMode = GetAsmMode();
        m_e2AsmPhase = GetAsmPhase();
        m_e2AsmTarget = GetAsmTarget();
        m_e2AsmPowerW = asmPowerW;
        m_e2AsmEnergyJ = asmEnergyJ;
    }

    if (m_enableGymFileLogging && m_sendDu)
    {
        std::ofstream csv{};
        csv.open(m_duFileName.c_str(), std::ios_base::app);
        if (!csv.is_open())
        {
            NS_FATAL_ERROR("Can't open file " << m_duFileName.c_str());
        }

        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();
        std::string asmKpm = std::to_string(static_cast<unsigned>(GetAsmMode())) + "," +
                             std::to_string(static_cast<unsigned>(GetAsmPhase())) + "," +
                             std::to_string(static_cast<unsigned>(GetAsmTarget())) + "," +
                             std::to_string(asmPowerW);

        // the string is timestamp, ueImsiComplete, plmId, nrCellId, dlAvailablePrbs,
        // ulAvailablePrbs, qci , dlPrbUsage, ulPrbUsage, /*CellSpecificValues*/, /*
        // UESpecificValues */

        /*
          CellSpecificValues:
            TB.TotNbrDl.1, TB.TotNbrDlInitial, TB.TotNbrDlInitial.Qpsk, TB.TotNbrDlInitial.16Qam,
          TB.TotNbrDlInitial.64Qam, RRU.PrbUsedDl, TB.ErrTotalNbrDl.1,
          QosFlow.PdcpPduVolumeDL_Filter, CARR.PDSCHMCSDist.Bin1, CARR.PDSCHMCSDist.Bin2,
          CARR.PDSCHMCSDist.Bin3, CARR.PDSCHMCSDist.Bin4, CARR.PDSCHMCSDist.Bin5,
          CARR.PDSCHMCSDist.Bin6, L1M.RS-SINR.Bin34, L1M.RS-SINR.Bin46, L1M.RS-SINR.Bin58,
            L1M.RS-SINR.Bin70, L1M.RS-SINR.Bin82, L1M.RS-SINR.Bin94, L1M.RS-SINR.Bin127,
          DRB.BufferSize.Qos, DRB.MeanActiveUeDl
        */

        std::string to_print_cell =
            plmId + "," + std::to_string(nrCellId) + "," + std::to_string(dlAvailablePrbs) + "," +
            std::to_string(ulAvailablePrbs) + "," + std::to_string(qci) + "," +
            std::to_string(dlPrbUsage) + "," + std::to_string(ulPrbUsage) + "," +
            std::to_string(macPduCellSpecific) + "," + std::to_string(macPduInitialCellSpecific) +
            "," + std::to_string(macQpskCellSpecific) + "," + std::to_string(mac16QamCellSpecific) +
            "," + std::to_string(mac64QamCellSpecific) + "," +
            std::to_string((long)std::ceil(prbUtilizationDl)) + "," +
            std::to_string(macRetxCellSpecific) + "," + std::to_string(macVolumeCellSpecific) +
            "," + std::to_string(macMac04CellSpecific) + "," +
            std::to_string(macMac59CellSpecific) + "," + std::to_string(macMac1014CellSpecific) +
            "," + std::to_string(macMac1519CellSpecific) + "," +
            std::to_string(macMac2024CellSpecific) + "," + std::to_string(macMac2529CellSpecific) +
            "," + std::to_string(macSinrBin1CellSpecific) + "," +
            std::to_string(macSinrBin2CellSpecific) + "," +
            std::to_string(macSinrBin3CellSpecific) + "," +
            std::to_string(macSinrBin4CellSpecific) + "," +
            std::to_string(macSinrBin5CellSpecific) + "," +
            std::to_string(macSinrBin6CellSpecific) + "," +
            std::to_string(macSinrBin7CellSpecific) + "," +
            std::to_string(rlcBufferOccupCellSpecific) + "," + std::to_string(ueMap.size());

        /*
          UESpecificValues:

              TB.TotNbrDl.1.UEID, TB.TotNbrDlInitial.UEID, TB.TotNbrDlInitial.Qpsk.UEID,
          TB.TotNbrDlInitial.16Qam.UEID,TB.TotNbrDlInitial.64Qam.UEID, TB.ErrTotalNbrDl.1.UEID,
          QosFlow.PdcpPduVolumeDL_Filter.UEID, RRU.PrbUsedDl.UEID, CARR.PDSCHMCSDist.Bin1.UEID,
          CARR.PDSCHMCSDist.Bin2.UEID, CARR.PDSCHMCSDist.Bin3.UEID, CARR.PDSCHMCSDist.Bin5.UEID,
          CARR.PDSCHMCSDist.Bin6.UEID, L1M.RS-SINR.Bin34.UEID, L1M.RS-SINR.Bin46.UEID,
          L1M.RS-SINR.Bin58.UEID, L1M.RS-SINR.Bin70.UEID, L1M.RS-SINR.Bin82.UEID,
          L1M.RS-SINR.Bin94.UEID, L1M.RS-SINR.Bin127.UEID, DRB.BufferSize.Qos.UEID,
          DRB.UEThpDl.UEID,DRB.UEThpDlPdcpBased.UEID
        */

        for (auto ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string ueImsiComplete = GetImsiString(imsi);

            auto uePms = uePmStringDu.find(imsi)->second;

            std::string to_print = std::to_string(timestamp) + "," + ueImsiComplete + "," +
                                   to_print_cell + "," + uePms + "," + asmKpm + "\n";

            csv << to_print;
        }
        if (ueMap.empty())
        {
            // OFF evicts every UE. Still emit one cell row so ASM state and power stay in the KPM.
            std::string ueZeros =
                "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
            csv << std::to_string(timestamp) << ",0," << to_print_cell << "," << ueZeros << ","
                << asmKpm << "\n";
        }
        csv.close();
    }
    return nullptr;
}

void
MmWaveEnbNetDevice::SendGnbE2Reports(std::string plmId, std::string gnbId)
{
    if (!m_e2term || !m_e2Subscribed)
    {
        return;
    }

    auto sendIndication = [&](Ptr<KpmIndicationMessage> msg) {
        if (!msg || !msg->m_buffer)
        {
            return;
        }
        Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
        if (!header)
        {
            return;
        }
        E2AP_PDU* pdu = (E2AP_PDU*)calloc(1, sizeof(E2AP_PDU));
        encoding::generate_e2apv1_indication_request_parameterized(
            pdu,
            m_e2ReportParams.requestorId,
            m_e2ReportParams.instanceId,
            m_e2ReportParams.ranFuncionId,
            m_e2ReportParams.actionId,
            1,
            (uint8_t*)header->m_buffer,
            header->m_size,
            (uint8_t*)msg->m_buffer,
            msg->m_size);
        m_e2term->SendE2Message(pdu);
    };

    if (!m_e2UeKpm.empty())
    {
        Ptr<MmWaveIndicationMessageHelper> ueHelper = Create<MmWaveIndicationMessageHelper>(
            IndicationMessageHelper::IndicationMessageType::gNB,
            false,
            m_reducedPmValues);
        for (auto& item : m_e2UeKpm)
        {
            const E2UeKpm& ue = item.second;
            ueHelper->AddgNBUeItem(ue.imsiComplete,
                                   ue.numDrb,
                                   0,
                                   ue.txPdcpPduBytesNrRlc,
                                   ue.txPdcpPduNrRlc,
                                   ue.macPduUe,
                                   ue.macPduInitialUe,
                                   ue.macQpsk,
                                   ue.mac16Qam,
                                   ue.mac64Qam,
                                   ue.macRetx,
                                   ue.macVolume,
                                   static_cast<long>(std::ceil(ue.macPrb)),
                                   ue.macMac04,
                                   ue.macMac59,
                                   ue.macMac1014,
                                   ue.macMac1519,
                                   ue.macMac2024,
                                   ue.macMac2529,
                                   ue.macSinrBin1,
                                   ue.macSinrBin2,
                                   ue.macSinrBin3,
                                   ue.macSinrBin4,
                                   ue.macSinrBin5,
                                   ue.macSinrBin6,
                                   ue.macSinrBin7,
                                   ue.rlcBufferOccup,
                                   ue.drbThrDlUeid,
                                   ue.sinrServ,
                                   ue.sinrServ3gpp,
                                   ue.servCell,
                                   ue.neighSinr[0],
                                   ue.neighSinr3gpp[0],
                                   ue.neighId[0],
                                   ue.neighSinr[1],
                                   ue.neighSinr3gpp[1],
                                   ue.neighId[1],
                                   ue.neighSinr[2],
                                   ue.neighSinr3gpp[2],
                                   ue.neighId[2],
                                   ue.neighSinr[3],
                                   ue.neighSinr3gpp[3],
                                   ue.neighId[3],
                                   ue.neighSinr[4],
                                   ue.neighSinr3gpp[4],
                                   ue.neighId[4],
                                   ue.neighSinr[5],
                                   ue.neighSinr3gpp[5],
                                   ue.neighId[5],
                                   ue.neighSinr[6],
                                   ue.neighSinr3gpp[6],
                                   ue.neighId[6],
                                   ue.neighSinr[7],
                                   ue.neighSinr3gpp[7],
                                   ue.neighId[7]);
        }
        sendIndication(ueHelper->CreateIndicationMessage("ue"));
    }

    if (m_e2CellValid)
    {
        Ptr<MmWaveIndicationMessageHelper> cellHelper = Create<MmWaveIndicationMessageHelper>(
            IndicationMessageHelper::IndicationMessageType::gNB,
            false,
            m_reducedPmValues);
        cellHelper->AddgNBCellItem(m_e2CellId,
                                   m_e2NumActiveUes,
                                   m_e2MacPdu,
                                   m_e2MacPduInitial,
                                   m_e2MacQpsk,
                                   m_e2Mac16Qam,
                                   m_e2Mac64Qam,
                                   m_e2PrbUtilizationDl,
                                   m_e2MacRetx,
                                   m_e2MacVolume,
                                   m_e2MacMac04,
                                   m_e2MacMac59,
                                   m_e2MacMac1014,
                                   m_e2MacMac1519,
                                   m_e2MacMac2024,
                                   m_e2MacMac2529,
                                   m_e2SinrBin1,
                                   m_e2SinrBin2,
                                   m_e2SinrBin3,
                                   m_e2SinrBin4,
                                   m_e2SinrBin5,
                                   m_e2SinrBin6,
                                   m_e2SinrBin7,
                                   m_e2RlcBuffer,
                                   m_e2ActiveUeDl,
                                   m_e2DlAvailablePrbs,
                                   m_e2UlAvailablePrbs,
                                   m_e2Qci,
                                   m_e2DlPrbUsage,
                                   m_e2UlPrbUsage,
                                   m_e2AsmMode,
                                   m_e2AsmPhase,
                                   m_e2AsmTarget,
                                   m_e2AsmPowerW,
                                   m_e2AsmEnergyJ);
        sendIndication(cellHelper->CreateIndicationMessage("cell"));
    }
}

void
MmWaveEnbNetDevice::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
    (void)params;
    std::string plmId = "111";
    std::string gnbId = std::to_string(m_cellId);

    NS_LOG_DEBUG("MmWaveEnbNetDevice " << m_cellId << " BuildAndSendMessage at time "
                                       << Simulator::Now().GetSeconds());

    m_e2UeKpm.clear();
    m_e2CellValid = false;

    if (m_sendCuUp || m_e2term)
    {
        BuildRicIndicationMessageCuUp(plmId);
    }
    if (m_sendCuCp || m_e2term)
    {
        BuildRicIndicationMessageCuCp(plmId);
    }
    if (m_sendDu || m_e2term)
    {
        BuildRicIndicationMessageDu(plmId, m_cellId);
    }
    SendGnbE2Reports(plmId, gnbId);

    if (m_e2term && !m_enableGymFileLogging)
    {
        Simulator::ScheduleWithContext(1,
                                       Seconds(m_e2Periodicity),
                                       &MmWaveEnbNetDevice::BuildAndSendReportMessage,
                                       this,
                                       m_e2ReportParams);
    }
    else
    {
        Simulator::Schedule(Seconds(m_e2Periodicity),
                            &MmWaveEnbNetDevice::BuildAndSendReportMessage,
                            this,
                            m_e2ReportParams);
    }
}

void
MmWaveEnbNetDevice::SetStartTime(uint64_t st)
{
    m_startTime = st;
}

uint64_t
MmWaveEnbNetDevice::GetStartTime()
{
    return m_startTime;
}

} // namespace mmwave
} // namespace ns3
