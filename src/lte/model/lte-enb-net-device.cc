/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 TELEMATICS LAB, DEE - Politecnico di Bari
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Giuseppe Piro  <g.piro@poliba.it>
 * Author: Marco Miozzo <mmiozzo@cttc.es> : Update to FF API Architecture
 * Author: Nicola Baldo <nbaldo@cttc.es>  : Integrated with new RRC and MAC architecture
 * Author: Danilo Abrignani <danilo.abrignani@unibo.it> : Integrated with new architecture - GSoC
 * 2015 - Carrier Aggregation
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *          Dual Connectivity functionalities
 *          Integration of ns-O-RAN
 *
 * Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                Integration of ns-O-RAN
 * 
 * Modified by: Andrea Lacava <thecave003@gmail.com>
 *                Integration of ns-O-RAN
 *                ns-O-RAN semaphore logic
 */

#include "encode_e2apv1.hpp"

#include <ns3/abort.h>
#include <ns3/callback.h>
#include <ns3/enum.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/llc-snap-header.h>
#include <ns3/log.h>
#include <ns3/lte-amc.h>
#include <ns3/lte-anr.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/config.h>
#include <semaphore.h>
#include <fcntl.h>
#include <ns3/lte-enb-mac.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-ffr-algorithm.h>
#include <ns3/lte-handover-algorithm.h>
#include <ns3/lte-indication-message-helper.h>
#include <ns3/lte-net-device.h>
#include <ns3/lte-radio-bearer-info.h>
#include <ns3/lte-rlc-am.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-um-lowlat.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/mc-enb-pdcp.h>
#include <ns3/node.h>
#include <ns3/object-factory.h>
#include <ns3/object-map.h>
#include <ns3/packet-burst.h>
#include <ns3/packet.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LteEnbNetDevice");

NS_OBJECT_ENSURE_REGISTERED(LteEnbNetDevice);

/**
 * KPM Subscription Request callback.
 * This function is triggered whenever a RIC Subscription Request for
 * the KPM RAN Function is received.
 *
 * \param pdu request message
 */
void
LteEnbNetDevice::KpmSubscriptionCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_DEBUG("Received RIC Subscription Request, cellId = " << m_cellId);

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
                                       &LteEnbNetDevice::BuildAndSendReportMessage,
                                       this,
                                       m_e2ReportParams);
    }
}

bool
LteEnbNetDevice::IsE2Subscribed() const
{
    return m_e2Subscribed.load();
}

std::string
LteEnbNetDevice::GetCurrentDirectory()
{
    NS_LOG_FUNCTION(this);
    int ret = std::system("pwd >path.txt");
    if (ret < 0)
    {
        NS_FATAL_ERROR("Unable to execute pwd command ");
    }

    std::string path;
    std::getline(std::ifstream("path.txt"), path);
    NS_LOG_INFO("I am running in " << path);
    return path;
}

void
LteEnbNetDevice::ReadControlFile()
{
    NS_LOG_INFO(Simulator::Now().GetMilliSeconds()
                << " I will try to read the control file " << m_controlFilename << " mode "
                << m_controlMode);
    std::string mode = NormalizeControlMode();
    bool hasControl = (m_controlFilename != "") || !mode.empty();
    if (!hasControl)
    {
        return;
    }

    if (m_useSemaphores)
    {
        sem_t* metricsReadySemaphore = sem_open(m_metricsReadySemaphoreName.c_str(), 0);
        NS_ABORT_MSG_IF(metricsReadySemaphore == SEM_FAILED,
                        "Error in opening the metrics semaphore, errno: " << strerror(errno));

        if (sem_post(metricsReadySemaphore) == -1)
        {
            NS_FATAL_ERROR("Error post named metrics semaphore: " << strerror(errno));
        }

        sem_close(metricsReadySemaphore);
        metricsReadySemaphore = nullptr;

        sem_t* controlSemaphore = sem_open(m_controlSemaphoreName.c_str(), 0);
        NS_ABORT_MSG_IF(controlSemaphore == SEM_FAILED,
                        "Error in opening the control semaphore, errno: " << strerror(errno));

        if (sem_wait(controlSemaphore) == -1)
        {
            NS_FATAL_ERROR("Error wait control control semaphore: " << strerror(errno));
        }

        sem_close(controlSemaphore);
        controlSemaphore = nullptr;
        // [nsoran-asm] Unblock mmWave gNBs so they apply asm_actions_for_ns3.csv now.
        m_gymControlReady();
    }

    std::string tsPath = TsControlPath();
    std::string esPath = EsControlPath();
    std::vector<std::string> flushed;

    // [nsoran-tses] ts: TS CSV only; es: ES CSV only (state + eviction);
    // tses: ES sets hoAllowed only, then TS places UEs, then leftover eviction.
    if (mode == "asm")
    {
        NS_LOG_INFO("ASM Gym handshake: mmWave applies ControlFileNameAsm");
        flushed.push_back(m_controlFilename);
    }
    else if (mode == "tses")
    {
        ApplyEsControlFile(esPath, false);
        std::set<uint64_t> tsHandedOver = ApplyTsControlFile(tsPath);
        m_rrc->EvictUsersFromSecondaryCell(tsHandedOver);
        flushed.push_back(esPath);
        flushed.push_back(tsPath);
    }
    else if (mode == "ts")
    {
        ApplyTsControlFile(tsPath);
        flushed.push_back(tsPath);
    }
    else if (mode == "es")
    {
        ApplyEsControlFile(esPath, true);
        flushed.push_back(esPath);
    }
    else if (mode == "qos")
    {
        ApplyQosControlFile(m_controlFilename);
        flushed.push_back(m_controlFilename);
    }
    else
    {
        NS_FATAL_ERROR("Unknown use case not implemented yet with filename: "
                       << m_controlFilename << " mode: " << m_controlMode);
    }

    if (!m_scheduleControlMessages)
    {
        for (const auto& path : flushed)
        {
            FlushControlFile(path);
        }
        NS_LOG_INFO("File flushed");
        Simulator::Schedule(Seconds(m_e2Periodicity), &LteEnbNetDevice::ReadControlFile, this);
    }
}

std::string
LteEnbNetDevice::NormalizeControlMode() const
{
    std::string mode = m_controlMode;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (!mode.empty())
    {
        return mode;
    }
    if (m_controlFilename.find("ts_actions_for_ns3.csv") != std::string::npos)
    {
        return "ts";
    }
    if (m_controlFilename.find("es_actions_for_ns3.csv") != std::string::npos)
    {
        return "es";
    }
    if (m_controlFilename.find("qos_actions.csv") != std::string::npos)
    {
        return "qos";
    }
    if (m_controlFilename.find("asm_actions_for_ns3.csv") != std::string::npos)
    {
        return "asm";
    }
    return "";
}

std::string
LteEnbNetDevice::TsControlPath() const
{
    if (m_controlFilename.find("ts_actions_for_ns3.csv") != std::string::npos)
    {
        return m_controlFilename;
    }
    return m_controlFilenameTs.empty() ? m_controlFilename : m_controlFilenameTs;
}

std::string
LteEnbNetDevice::EsControlPath() const
{
    if (m_controlFilename.find("es_actions_for_ns3.csv") != std::string::npos)
    {
        return m_controlFilename;
    }
    return m_controlFilenameEs.empty() ? m_controlFilename : m_controlFilenameEs;
}

void
LteEnbNetDevice::FlushControlFile(const std::string& path)
{
    if (path.empty())
    {
        return;
    }
    std::ofstream csvDelete{};
    csvDelete.open(path.c_str());
}

std::set<uint64_t>
LteEnbNetDevice::ApplyTsControlFile(const std::string& path)
{
    std::set<uint64_t> handedOver;
    if (path.empty())
    {
        return handedOver;
    }
    std::ifstream csv{};
    csv.open(path.c_str(), std::ifstream::in);
    if (!csv.is_open())
    {
        NS_FATAL_ERROR("Can't open file " << path);
    }
    std::string line;
    while (std::getline(csv, line))
    {
        if (line == "")
        {
            continue;
        }
        NS_LOG_INFO("Read handover command");
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ',');
        long long timestamp = std::stoll(cell);

        uint64_t imsi;
        std::getline(lineStream, cell, ',');
        imsi = std::stoi(cell);

        uint16_t targetCellId;
        std::getline(lineStream, cell, ',');
        targetCellId = std::stoi(cell);

        NS_LOG_INFO("Handover command for timestamp " << timestamp << " imsi " << imsi
                                                      << " targetCellId " << targetCellId);

        uint16_t rntiUe = m_rrc->GetRntiFromImsi(imsi);
        if (rntiUe == 0 || !m_rrc->HasUeManager(rntiUe))
        {
            NS_LOG_WARN("UE IMSI " << imsi << " has no RNTI/manager, ignoring TS HO");
            continue;
        }
        uint16_t sourceCellId = m_rrc->GetLastMmWaveCell(imsi);
        if (sourceCellId == 0)
        {
            sourceCellId = m_rrc->GetUeManager(rntiUe)->GetMmWaveCellId();
        }

        uint16_t resolved = m_rrc->ResolveTsHandoverTarget(imsi, targetCellId);
        if (resolved == 0)
        {
            NS_LOG_WARN("No allowed TS target for IMSI " << imsi << " requested " << targetCellId);
            continue;
        }
        if (sourceCellId == resolved)
        {
            NS_LOG_WARN("Source CellId and Target CellId are the same "
                        << unsigned(sourceCellId) << ", ignoring HO request");
            continue;
        }

        m_rrc->TakeUeHoControl(imsi);
        handedOver.insert(imsi);
        Simulator::ScheduleWithContext(1,
                                       Seconds(0),
                                       &LteEnbRrc::PerformHandoverToTargetCell,
                                       m_rrc,
                                       imsi,
                                       resolved);
    }
    csv.close();
    return handedOver;
}

void
LteEnbNetDevice::ApplyEsControlFile(const std::string& path, bool evictUsers)
{
    if (path.empty())
    {
        return;
    }
    std::ifstream csv{};
    csv.open(path.c_str(), std::ifstream::in);
    if (!csv.is_open())
    {
        NS_FATAL_ERROR("Can't open file " << path);
    }
    std::string line;
    long long timestamp{};
    uint8_t counter = 0;
    while (std::getline(csv, line))
    {
        if (line == "")
        {
            continue;
        }
        NS_LOG_INFO("Read evict users command");
        counter++;
        std::stringstream lineStream(line);
        std::string cell;

        std::getline(lineStream, cell, ',');
        timestamp = std::stoll(cell);

        uint16_t cellId;
        std::getline(lineStream, cell, ',');
        cellId = std::stoi(cell);

        bool hoAllowed;
        std::getline(lineStream, cell, ',');
        hoAllowed = std::stoi(cell);

        NS_LOG_INFO("Set allowed command with timestamp " << timestamp << " cellId " << cellId
                                                          << " hoAllowed " << hoAllowed);

        if (!m_scheduleControlMessages)
        {
            m_rrc->SetSecondaryCellHandoverAllowedStatus(cellId, hoAllowed);
        }
        else
        {
            Simulator::Schedule(MilliSeconds(timestamp),
                                &LteEnbRrc::SetSecondaryCellHandoverAllowedStatus,
                                m_rrc,
                                cellId,
                                hoAllowed);
        }
    }
    csv.close();

    if (evictUsers && counter > 0)
    {
        if (!m_scheduleControlMessages)
        {
            m_rrc->EvictUsersFromSecondaryCell();
        }
        else
        {
            Simulator::Schedule(MilliSeconds(timestamp + 0.001),
                                &LteEnbRrc::EvictUsersFromSecondaryCell,
                                m_rrc,
                                std::set<uint64_t>{});
        }
    }
}

void
LteEnbNetDevice::ApplyQosControlFile(const std::string& path)
{
    if (path.empty())
    {
        return;
    }
    std::ifstream csv{};
    csv.open(path.c_str(), std::ifstream::in);
    if (!csv.is_open())
    {
        NS_FATAL_ERROR("Can't open file " << path);
    }
    std::string line;
    std::unordered_map<uint16_t, double> uePercentages{};
    NS_LOG_INFO("Read QoS command");
    while (std::getline(csv, line))
    {
        if (line == "")
        {
            continue;
        }

        std::stringstream lineStream(line);
        std::string data;

        std::getline(lineStream, data, ',');
        long long timestamp = std::stoll(data);

        uint16_t ueId;
        std::getline(lineStream, data, ',');
        ueId = std::stoi(data);

        double uePerc;
        std::getline(lineStream, data, ',');
        uePerc = std::stof(data);
        if (uePerc < 0 || uePerc > 1)
        {
            NS_LOG_ERROR("Wrong value for ueid " << uePerc << " percentage ");
        }
        else
        {
            NS_LOG_INFO("Set ue percentage command with timestamp " << timestamp << " ueId "
                                                                    << ueId << "percentage"
                                                                    << uePerc);
            uePercentages.insert({ueId, uePerc});
        }
    }
    csv.close();

    auto ueMap = m_rrc->GetUeMap();
    for (std::pair<uint64_t, double> uePercentage : uePercentages)
    {
        uint16_t ueId = m_rrc->GetRntiFromImsi(uePercentage.first);
        if (ueMap.find(ueId) == ueMap.end())
        {
            NS_LOG_ERROR(ueId << " not found in UeMap");
            NS_FATAL_ERROR("Wrong UE RNTI passed by the controller, aborting...");
        }
        this->SetUeQoS(ueId, uePercentage.second);
    }
}

void
LteEnbNetDevice::SetUeQoS(uint16_t ueId, double percentage)
{
    auto ueManager = m_rrc->GetUeManager(ueId);
    auto drbMap = ueManager->GetDrbMap();
    for (auto drb : drbMap)
    {
        auto dataBearer = drb.second;
        Ptr<McEnbPdcp> pdcp = DynamicCast<McEnbPdcp>(dataBearer->m_pdcp);
        if (pdcp)
        {
            NS_LOG_INFO(Simulator::Now().GetMilliSeconds()
                        << ": About to set pecentage " << percentage
                        << " on UE connectes to eNB with RNTI " << ueId);
            pdcp->SetAttribute("perPckToLTE", DoubleValue(percentage));
        }
        else
        {
            NS_LOG_WARN("pdcp not found");
        }
    }
}

double
LteEnbNetDevice::GetUeQoS(uint16_t rnti)
{
    auto ueManager = m_rrc->GetUeManager(rnti);
    auto drbMap = ueManager->GetDrbMap();
    DoubleValue percentage;
    double ueQoS;
    for (auto drb : drbMap)
    {
        auto dataBearer = drb.second;
        Ptr<McEnbPdcp> pdcp = DynamicCast<McEnbPdcp>(dataBearer->m_pdcp);
        if (pdcp)
        {
            pdcp->GetAttribute("perPckToLTE", percentage);
            ueQoS = percentage.Get();
            return ueQoS;
        }
        else
        {
            NS_LOG_UNCOND("pdcp not found");
        }
    }
    return -1.0; // This should never happen
}

double
LteEnbNetDevice::GetUeQoS(uint64_t imsi)
{
    return GetUeQoS(m_rrc->GetRntiFromImsi(imsi));
}

void
LteEnbNetDevice::ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_DEBUG("LteEnbNetDevice::ControlMessageReceivedCallback: Received RIC Control Message");

    Ptr<RicControlMessage> controlMessage = Create<RicControlMessage>(sub_req_pdu);
    NS_LOG_INFO("After RicControlMessage::RicControlMessage constructor");
    NS_LOG_INFO("Request type " << controlMessage->m_requestType);
    switch (controlMessage->m_requestType)
    {
    case RicControlMessage::ControlMessageRequestIdType::TS: {
        NS_LOG_INFO("TS, do the handover");
        // do handover
        uint64_t imsi = controlMessage->GetUeId();
        uint16_t targetCellId = std::stoi(controlMessage->GetSecondaryCellIdHO());
        NS_LOG_INFO("Imsi Decoded: " << imsi);
        NS_LOG_INFO("Target Cell id " << targetCellId);
        m_rrc->TakeUeHoControl(imsi);
        if (!m_enableGymFileLogging)
        {
            Simulator::ScheduleWithContext(1,
                                           Seconds(0),
                                           &LteEnbRrc::PerformHandoverToTargetCell,
                                           m_rrc,
                                           imsi,
                                           targetCellId);
        }
        else
        {
            Simulator::Schedule(Seconds(0),
                                &LteEnbRrc::PerformHandoverToTargetCell,
                                m_rrc,
                                imsi,
                                targetCellId);
        }
        break;
    }
    case RicControlMessage::ControlMessageRequestIdType::QoS: {
        // use SetUeQoS()
        NS_FATAL_ERROR("For QoS use file-based control.");
        break;
    }
    default: {
        NS_LOG_ERROR("Unrecognized id type of Ric Control Message");
        break;
    }
    }
}

TypeId
LteEnbNetDevice::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::LteEnbNetDevice")
            .SetParent<LteNetDevice>()
            .AddConstructor<LteEnbNetDevice>()
            .AddAttribute("LteEnbRrc",
                          "The RRC associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_rrc),
                          MakePointerChecker<LteEnbRrc>())
            .AddAttribute("LteHandoverAlgorithm",
                          "The handover algorithm associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_handoverAlgorithm),
                          MakePointerChecker<LteHandoverAlgorithm>())
            .AddAttribute(
                "LteAnr",
                "The automatic neighbour relation function associated to this EnbNetDevice",
                PointerValue(),
                MakePointerAccessor(&LteEnbNetDevice::m_anr),
                MakePointerChecker<LteAnr>())
            .AddAttribute("LteFfrAlgorithm",
                          "The FFR algorithm associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_ffrAlgorithm),
                          MakePointerChecker<LteFfrAlgorithm>())
            .AddAttribute("LteEnbComponentCarrierManager",
                          "The RRC associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_componentCarrierManager),
                          MakePointerChecker<LteEnbComponentCarrierManager>())
            .AddAttribute("ComponentCarrierMap",
                          "List of component carriers.",
                          ObjectMapValue(),
                          MakeObjectMapAccessor(&LteEnbNetDevice::m_ccMap),
                          MakeObjectMapChecker<ComponentCarrierEnb>())
            .AddAttribute(
                "UlBandwidth",
                "Uplink Transmission Bandwidth Configuration in number of Resource Blocks",
                UintegerValue(100),
                MakeUintegerAccessor(&LteEnbNetDevice::SetUlBandwidth,
                                     &LteEnbNetDevice::GetUlBandwidth),
                MakeUintegerChecker<uint8_t>())
            .AddAttribute(
                "DlBandwidth",
                "Downlink Transmission Bandwidth Configuration in number of Resource Blocks",
                UintegerValue(100),
                MakeUintegerAccessor(&LteEnbNetDevice::SetDlBandwidth,
                                     &LteEnbNetDevice::GetDlBandwidth),
                MakeUintegerChecker<uint8_t>())
            .AddAttribute("CellId",
                          "Cell Identifier",
                          UintegerValue(0),
                          MakeUintegerAccessor(&LteEnbNetDevice::m_cellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("DlEarfcn",
                          "Downlink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                          "as per 3GPP 36.101 Section 5.7.3. ",
                          UintegerValue(100),
                          MakeUintegerAccessor(&LteEnbNetDevice::m_dlEarfcn),
                          MakeUintegerChecker<uint32_t>(0, 262143))
            .AddAttribute("UlEarfcn",
                          "Uplink E-UTRA Absolute Radio Frequency Channel Number (EARFCN) "
                          "as per 3GPP 36.101 Section 5.7.3. ",
                          UintegerValue(18100),
                          MakeUintegerAccessor(&LteEnbNetDevice::m_ulEarfcn),
                          MakeUintegerChecker<uint32_t>(0, 262143))
            .AddAttribute(
                "CsgId",
                "The Closed Subscriber Group (CSG) identity that this eNodeB belongs to",
                UintegerValue(0),
                MakeUintegerAccessor(&LteEnbNetDevice::SetCsgId, &LteEnbNetDevice::GetCsgId),
                MakeUintegerChecker<uint32_t>())
            .AddAttribute(
                "CsgIndication",
                "If true, only UEs which are members of the CSG (i.e. same CSG ID) "
                "can gain access to the eNodeB, therefore enforcing closed access mode. "
                "Otherwise, the eNodeB operates as a non-CSG cell and implements open access mode.",
                BooleanValue(false),
                MakeBooleanAccessor(&LteEnbNetDevice::SetCsgIndication,
                                    &LteEnbNetDevice::GetCsgIndication),
                MakeBooleanChecker())
            .AddAttribute("E2Termination",
                          "The E2 termination object associated to this node",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::SetE2Termination,
                                              &LteEnbNetDevice::GetE2Termination),
                          MakePointerChecker<E2Termination>())
            .AddAttribute("E2PdcpCalculator",
                          "The PDCP calculator object for E2 reporting",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_e2PdcpStatsCalculator),
                          MakePointerChecker<mmwave::MmWaveBearerStatsCalculator>())
            .AddAttribute("E2RlcCalculator",
                          "The RLC calculator object for E2 reporting",
                          PointerValue(),
                          MakePointerAccessor(&LteEnbNetDevice::m_e2RlcStatsCalculator),
                          MakePointerChecker<mmwave::MmWaveBearerStatsCalculator>())
            .AddAttribute("E2Periodicity",
                          "Periodicity of E2 reporting (value in seconds)",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&LteEnbNetDevice::m_e2Periodicity),
                          MakeDoubleChecker<double>())
            .AddAttribute("EnableCuUpReport",
                          "If true, include the CU-UP report. "
                          "Appended to cu-up-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_sendCuUp),
                          MakeBooleanChecker())
            .AddAttribute("EnableCuCpReport",
                          "If true, include the CU-CP report. "
                          "Appended to cu-cp-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_sendCuCp),
                          MakeBooleanChecker())
            .AddAttribute("EnableDuReport",
                          "If true, include the DU report. "
                          "Appended to du-cell-<id>.txt when EnableGYMFileLogging is true.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_sendDu),
                          MakeBooleanChecker())
            .AddAttribute("ReducedPmValues",
                          "If true, send only a subset of pmValues",
                          BooleanValue(false),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_reducedPmValues),
                          MakeBooleanChecker())
            .AddAttribute("EnableGYMFileLogging",
                          "If true, write the enabled CU-UP, CU-CP, and DU reports to csv files.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_enableGymFileLogging),
                          MakeBooleanChecker())
            .AddAttribute(
                "ControlFileName",
                "Filename for the stand alone control mode. The file is deleted after every read."
                "Format should correspond to the particular use case:\n"
                "TS: Contains multiple lines with ts, imsi, targetCellId\n"
                "QoS: Contains multiple lines with ts, imsi, qos\n"
                "EE: Contains multiple lines with ts, cellId, action",
                StringValue(""),
                MakeStringAccessor(&LteEnbNetDevice::m_controlFilename),
                MakeStringChecker())
            .AddAttribute("ControlMode",
                          "Control use case: ts, es, tses, qos, asm. Empty infers from ControlFileName.",
                          StringValue(""),
                          MakeStringAccessor(&LteEnbNetDevice::m_controlMode),
                          MakeStringChecker())
            .AddAttribute("ControlFileNameTs",
                          "TS handover CSV (timestamp,imsi,targetCellId). Used for ts and tses.",
                          StringValue("ts_actions_for_ns3.csv"),
                          MakeStringAccessor(&LteEnbNetDevice::m_controlFilenameTs),
                          MakeStringChecker())
            .AddAttribute("ControlFileNameEs",
                          "ES cell on/off CSV (timestamp,cellId,hoAllowed). Used for es and tses.",
                          StringValue("es_actions_for_ns3.csv"),
                          MakeStringAccessor(&LteEnbNetDevice::m_controlFilenameEs),
                          MakeStringChecker())
            .AddAttribute("ScheduleControlMessages",
                          "If true, the control action will be scheduled at the given timestamp in "
                          "milliseconds.\n"
                          "If false, the control action will be executed when parsed.\n",
                          BooleanValue(false),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_scheduleControlMessages),
                          MakeBooleanChecker())
            .AddAttribute("UseSemaphores",
                          "If true, we use the semaphores to handle communication with an external "
                          "environment",
                          BooleanValue(false),
                          MakeBooleanAccessor(&LteEnbNetDevice::m_useSemaphores),
                          MakeBooleanChecker())
            .AddTraceSource("GymControlReady",
                            "[nsoran-asm] Fired after the Gym control semaphore wait.",
                            MakeTraceSourceAccessor(&LteEnbNetDevice::m_gymControlReady),
                            "ns3::EmptyCallback");
    return tid;
}

LteEnbNetDevice::LteEnbNetDevice()
    : m_isConstructed(false),
      m_isConfigured(false),
      m_anr(0),
      m_componentCarrierManager(0),
      m_isReportingEnabled(false),
      m_reducedPmValues(false),
      m_enableGymFileLogging(false),
      m_e2Subscribed(false),
      m_useSemaphores(false),
      m_cuUpFileName(),
      m_cuCpFileName(),
      m_duFileName(),      // [nsoran-qos]
      m_duTraceConnected(false)
{
    NS_LOG_FUNCTION(this);
}

LteEnbNetDevice::~LteEnbNetDevice(void)
{
    NS_LOG_FUNCTION(this);
}

void
LteEnbNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);

    m_rrc->Dispose();
    m_rrc = 0;

    m_handoverAlgorithm->Dispose();
    m_handoverAlgorithm = 0;

    if (m_anr)
    {
        m_anr->Dispose();
        m_anr = 0;
    }
    m_componentCarrierManager->Dispose();
    m_componentCarrierManager = 0;
    // ComponentCarrierEnb::DoDispose() will call DoDispose
    // of its PHY, MAC, FFR and scheduler instance
    for (uint32_t i = 0; i < m_ccMap.size(); i++)
    {
        m_ccMap.at(i)->Dispose();
        m_ccMap.at(i) = 0;
    }

    LteNetDevice::DoDispose();
}

Ptr<LteEnbMac>
LteEnbNetDevice::GetMac() const
{
    return m_ccMap.at(0)->GetMac();
}

Ptr<LteEnbPhy>
LteEnbNetDevice::GetPhy() const
{
    return m_ccMap.at(0)->GetPhy();
}

Ptr<LteEnbMac>
LteEnbNetDevice::GetMac(uint8_t index)
{
    return m_ccMap.at(index)->GetMac();
}

Ptr<LteEnbPhy>
LteEnbNetDevice::GetPhy(uint8_t index)
{
    return m_ccMap.at(index)->GetPhy();
}

Ptr<LteEnbRrc>
LteEnbNetDevice::GetRrc() const
{
    return m_rrc;
}

Ptr<LteEnbComponentCarrierManager>
LteEnbNetDevice::GetComponentCarrierManager() const
{
    return m_componentCarrierManager;
}

uint16_t
LteEnbNetDevice::GetCellId() const
{
    return m_cellId;
}

bool
LteEnbNetDevice::HasCellId(uint16_t cellId) const
{
    for (auto& it : m_ccMap)
    {
        if (it.second->GetCellId() == cellId)
        {
            return true;
        }
    }
    return false;
}

uint8_t
LteEnbNetDevice::GetUlBandwidth() const
{
    return m_ulBandwidth;
}

void
LteEnbNetDevice::SetUlBandwidth(uint8_t bw)
{
    NS_LOG_FUNCTION(this << uint16_t(bw));
    switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
        m_ulBandwidth = bw;
        break;

    default:
        NS_FATAL_ERROR("invalid bandwidth value " << (uint16_t)bw);
        break;
    }
}

uint8_t
LteEnbNetDevice::GetDlBandwidth() const
{
    return m_dlBandwidth;
}

void
LteEnbNetDevice::SetDlBandwidth(uint8_t bw)
{
    NS_LOG_FUNCTION(this << uint16_t(bw));
    switch (bw)
    {
    case 6:
    case 15:
    case 25:
    case 50:
    case 75:
    case 100:
        m_dlBandwidth = bw;
        break;

    default:
        NS_FATAL_ERROR("invalid bandwidth value " << (uint16_t)bw);
        break;
    }
}

uint32_t
LteEnbNetDevice::GetDlEarfcn() const
{
    return m_dlEarfcn;
}

void
LteEnbNetDevice::SetDlEarfcn(uint32_t earfcn)
{
    NS_LOG_FUNCTION(this << earfcn);
    m_dlEarfcn = earfcn;
}

uint32_t
LteEnbNetDevice::GetUlEarfcn() const
{
    return m_ulEarfcn;
}

void
LteEnbNetDevice::SetUlEarfcn(uint32_t earfcn)
{
    NS_LOG_FUNCTION(this << earfcn);
    m_ulEarfcn = earfcn;
}

uint32_t
LteEnbNetDevice::GetCsgId() const
{
    return m_csgId;
}

void
LteEnbNetDevice::SetCsgId(uint32_t csgId)
{
    NS_LOG_FUNCTION(this << csgId);
    m_csgId = csgId;
    UpdateConfig(); // propagate the change to RRC level
}

bool
LteEnbNetDevice::GetCsgIndication() const
{
    return m_csgIndication;
}

void
LteEnbNetDevice::SetCsgIndication(bool csgIndication)
{
    NS_LOG_FUNCTION(this << csgIndication);
    m_csgIndication = csgIndication;
    UpdateConfig(); // propagate the change to RRC level
}

std::map<uint8_t, Ptr<ComponentCarrierEnb>>
LteEnbNetDevice::GetCcMap()
{
    return m_ccMap;
}

void
LteEnbNetDevice::SetCcMap(std::map<uint8_t, Ptr<ComponentCarrierEnb>> ccm)
{
    NS_ASSERT_MSG(!m_isConfigured, "attempt to set CC map after configuration");
    m_ccMap = ccm;
}

void
LteEnbNetDevice::DoInitialize(void)
{
    NS_LOG_FUNCTION(this);
    m_isConstructed = true;
    UpdateConfig();
    std::map<uint8_t, Ptr<ComponentCarrierEnb>>::iterator it;
    for (it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
    {
        it->second->Initialize();
    }
    m_rrc->Initialize();
    m_componentCarrierManager->Initialize();
    m_handoverAlgorithm->Initialize();

    if (m_anr)
    {
        m_anr->Initialize();
    }

    m_ffrAlgorithm->Initialize();
}

void
LteEnbNetDevice::ReportCurrentCellRsrpSinr(Ptr<LteEnbNetDevice> netDev,
                                           std::string context,
                                           uint16_t cellId,
                                           uint16_t rnti,
                                           double rsrp,
                                           double sinr,
                                           uint8_t componentCarrierId)
{
    uint64_t imsi = netDev->GetRrc()->GetImsiFromRnti(rnti);
    // NS_LOG_UNCOND ("RegisterNewSinrReadingCallback rnti" << std::to_string(rnti)<< " imsi "<<
    // std::to_string(imsi));
    netDev->RegisterNewSinrReading(imsi, cellId, sinr);
}

void
LteEnbNetDevice::RegisterNewSinrReading(uint64_t imsi, uint16_t cellId, long double sinr)
{
    if (!m_sendCuCp)
    {
        return;
    }

    // Create key
    ImsiCellIdPair_t imsiCid{imsi, cellId};

    // We only need to save the last value, so we do not care about overwriting or not
    m_l3sinrMap[imsiCid] = sinr;

    NS_LOG_LOGIC(Simulator::Now().GetSeconds()
                 << " enbdev " << m_cellId << " UE " << imsi << " report for " << cellId << " SINR "
                 << m_l3sinrMap[imsiCid]);
}

bool
LteEnbNetDevice::Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                        protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber
                                            << ", only IPv4 and IPv6 are supported");
    return m_rrc->SendData(packet);
}

void
LteEnbNetDevice::UpdateConfig(void)
{
    NS_LOG_FUNCTION(this);

    if (m_isConstructed)
    {
        if (!m_isConfigured)
        {
            NS_LOG_LOGIC(this << " Configure cell " << m_cellId);
            // we have to make sure that this function is called only once
            NS_ASSERT(!m_ccMap.empty());
            m_rrc->ConfigureCell(m_ccMap);
            m_isConfigured = true;
        }

        NS_LOG_LOGIC(this << " Updating SIB1 of cell " << m_cellId << " with CSG ID " << m_csgId
                          << " and CSG indication " << m_csgIndication);
        m_rrc->SetCsgId(m_csgId, m_csgIndication);

        if (m_e2term)
        {
            NS_LOG_DEBUG("E2sim start in cell " << m_cellId << " gym file logging "
                                                << m_enableGymFileLogging);

            Simulator::Schedule(MicroSeconds(0), &E2Termination::Start, m_e2term);
            if (m_enableGymFileLogging)
            { // give some time for the simulation to start, TODO check value
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
                       "DRB.PdcpPduNbrDl.Qos.UEID(txPdcpPduNrRlc)\n";
                csv.close();

                m_cuCpFileName = "cu-cp-cell-" + std::to_string(m_cellId) + ".txt";
                csv.open(m_cuCpFileName.c_str());
                csv << "timestamp,ueImsiComplete,numActiveUes,RRC.ConnMean,DRB.EstabSucc.5QI.UEID "
                       "(numDrb),"
                       "DRB.RelActNbr.5QI.UEID (0),eNB id,sameCellSinr,"
                       "sameCellSinr 3gpp encoded\n";
                csv.close();

                // [nsoran-qos] LTE had CU-UP/CU-CP files only. Mirror mmWave du-cell-*.txt for PRB KPMs.
                if (m_sendDu)
                {
                    m_duFileName = "du-cell-" + std::to_string(m_cellId) + ".txt";
                    csv.open(m_duFileName.c_str());
                    csv << "timestamp,ueImsiComplete,plmId,nrCellId,dlAvailablePrbs,"
                           "ulAvailablePrbs,qci,dlPrbUsage,ulPrbUsage,"
                           "TB.TotNbrDl.1,TB.TotNbrDlInitial,TB.TotNbrDlInitial.Qpsk,"
                           "TB.TotNbrDlInitial.16Qam,TB.TotNbrDlInitial.64Qam,RRU.PrbUsedDl,"
                           "TB.ErrTotalNbrDl.1,QosFlow.PdcpPduVolumeDL_Filter,"
                           "CARR.PDSCHMCSDist.Bin1,CARR.PDSCHMCSDist.Bin2,CARR.PDSCHMCSDist.Bin3,"
                           "CARR.PDSCHMCSDist.Bin4,CARR.PDSCHMCSDist.Bin5,CARR.PDSCHMCSDist.Bin6,"
                           "L1M.RS-SINR.Bin34,L1M.RS-SINR.Bin46,L1M.RS-SINR.Bin58,"
                           "L1M.RS-SINR.Bin70,L1M.RS-SINR.Bin82,L1M.RS-SINR.Bin94,L1M.RS-SINR.Bin127,"
                           "DRB.BufferSize.Qos,DRB.MeanActiveUeDl,"
                           "TB.TotNbrDl.1.UEID,TB.TotNbrDlInitial.UEID,TB.TotNbrDlInitial.Qpsk.UEID,"
                           "TB.TotNbrDlInitial.16Qam.UEID,TB.TotNbrDlInitial.64Qam.UEID,"
                           "TB.ErrTotalNbrDl.1.UEID,QosFlow.PdcpPduVolumeDL_Filter.UEID,"
                           "RRU.PrbUsedDl.UEID,CARR.PDSCHMCSDist.Bin1.UEID,"
                           "CARR.PDSCHMCSDist.Bin2.UEID,CARR.PDSCHMCSDist.Bin3.UEID,"
                           "CARR.PDSCHMCSDist.Bin4.UEID,CARR.PDSCHMCSDist.Bin5.UEID,"
                           "CARR.PDSCHMCSDist.Bin6.UEID,L1M.RS-SINR.Bin34.UEID,"
                           "L1M.RS-SINR.Bin46.UEID,L1M.RS-SINR.Bin58.UEID,L1M.RS-SINR.Bin70.UEID,"
                           "L1M.RS-SINR.Bin82.UEID,L1M.RS-SINR.Bin94.UEID,L1M.RS-SINR.Bin127.UEID,"
                           "DRB.BufferSize.Qos.UEID,DRB.UEThpDl.UEID,DRB.UEThpDlPdcpBased.UEID\n";
                    csv.close();
                    ConnectDuMacTrace();
                }

                Simulator::Schedule(MicroSeconds(500),
                                    &LteEnbNetDevice::BuildAndSendReportMessage,
                                    this,
                                    E2Termination::RicSubscriptionRequest_rval_s{});
                m_isReportingEnabled = true;
                NS_LOG_INFO("About to schedule control file function");
                if (m_useSemaphores)
                {
                    std::string currentDirectory = GetCurrentDirectory();
                    // Extract folder name
                    size_t lastDelimiterPos = currentDirectory.find_last_of('/');
                    m_metricsReadySemaphoreName = "/sem_metrics_";
                    m_controlSemaphoreName = "/sem_control_";
                    if (lastDelimiterPos != std::string::npos)
                    {
                        m_metricsReadySemaphoreName +=
                            currentDirectory.substr(lastDelimiterPos + 1);
                        m_controlSemaphoreName += currentDirectory.substr(lastDelimiterPos + 1);
                    }
                    else
                    {
                        m_metricsReadySemaphoreName += currentDirectory;
                        m_controlSemaphoreName += currentDirectory;
                    }

                    NS_LOG_INFO("Name of semaphore for metrics is "
                                << m_metricsReadySemaphoreName.c_str());
                    NS_LOG_INFO("Name of semaphore for control is "
                                << m_controlSemaphoreName.c_str());
                                
                    // Ensure semaphores are created
                    int initial_count = 0;
                    sem_t* metricsReadySemaphore = sem_open(m_metricsReadySemaphoreName.c_str(),
                                                            O_CREAT,
                                                            S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
                                                            initial_count);
                    NS_ABORT_MSG_IF(
                        metricsReadySemaphore == SEM_FAILED,
                        "Error in opening the metrics semaphore, errno: " << strerror(errno));
                    sem_close(metricsReadySemaphore);
                    metricsReadySemaphore = nullptr;

                    sem_t* controlSemaphore = sem_open(m_controlSemaphoreName.c_str(),
                                                       O_CREAT,
                                                       S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
                                                       initial_count);
                    NS_ABORT_MSG_IF(
                        controlSemaphore == SEM_FAILED,
                        "Error in opening the control semaphore, errno: " << strerror(errno));
                    sem_close(controlSemaphore);
                    controlSemaphore = nullptr;

                    // This is needed when the output of the simulation and the control file are
                    // saved in a different folder from the ns-3 one
                    if (!m_controlFilename.empty() && m_controlFilename[0] != '/')
                    {
                        m_controlFilename = currentDirectory + '/' + m_controlFilename;
                    }
                    if (!m_controlFilenameTs.empty() && m_controlFilenameTs[0] != '/')
                    {
                        m_controlFilenameTs = currentDirectory + '/' + m_controlFilenameTs;
                    }
                    if (!m_controlFilenameEs.empty() && m_controlFilenameEs[0] != '/')
                    {
                        m_controlFilenameEs = currentDirectory + '/' + m_controlFilenameEs;
                    }
                    NS_LOG_INFO("The filename of the control file is " << m_controlFilename
                                                                       << " mode " << m_controlMode
                                                                       << " ts " << m_controlFilenameTs
                                                                       << " es " << m_controlFilenameEs);
                }

                // Set the first Control Action to happen after 5 ms to the first m_e2Periodicity
                Time e2ControlPeriodicity = Seconds(m_e2Periodicity) + MilliSeconds(5);
                Simulator::Schedule(e2ControlPeriodicity, &LteEnbNetDevice::ReadControlFile, this);
            }

            // Regardless the offline or online mode for reporting the files, we always want to
            // register the mean of RRC UEs
            Simulator::Schedule(MilliSeconds(10), &LteEnbNetDevice::SaveActiveUes, this);
        }
    }
    else
    {
        /*
         * Lower layers are not ready yet, so do nothing now and expect
         * ``DoInitialize`` to re-invoke this function.
         */
    }
}

Ptr<E2Termination>
LteEnbNetDevice::GetE2Termination() const
{
    return m_e2term;
}

void
LteEnbNetDevice::SetE2Termination(Ptr<E2Termination> e2term)
{
    m_e2term = e2term;

    NS_LOG_DEBUG("Register E2SM");

    if (m_e2term)
    {
        Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription>(0);
        e2term->RegisterKpmCallbackToE2Sm(
            2,
            kpmFd,
            std::bind(&LteEnbNetDevice::KpmSubscriptionCallback, this, std::placeholders::_1));

        Ptr<RicControlFunctionDescription> ricCtrlFd = Create<RicControlFunctionDescription>();
        e2term->RegisterSmCallbackToE2Sm(3,
                                         ricCtrlFd,
                                         std::bind(&LteEnbNetDevice::ControlMessageReceivedCallback,
                                                   this,
                                                   std::placeholders::_1));
    }
}

std::string
LteEnbNetDevice::GetImsiString(uint64_t imsi)
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
LteEnbNetDevice::BuildRicIndicationHeader(std::string plmId, std::string gnbId, uint16_t nrCellId)
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
            Create<KpmIndicationHeader>(KpmIndicationHeader::GlobalE2nodeType::eNB, headerValues);

        return header;
    }
    else
    {
        return nullptr;
    }
}

Ptr<KpmIndicationMessage>
LteEnbNetDevice::BuildRicIndicationMessageCuUp(std::string plmId)
{
    (void)plmId;
    Ptr<LteIndicationMessageHelper> ueHelper;
    Ptr<LteIndicationMessageHelper> cellHelper;
    if (m_e2term)
    {
        ueHelper = Create<LteIndicationMessageHelper>(
            IndicationMessageHelper::IndicationMessageType::eNB,
            false,
            m_reducedPmValues);
        cellHelper = Create<LteIndicationMessageHelper>(
            IndicationMessageHelper::IndicationMessageType::eNB,
            false,
            m_reducedPmValues);
    }

    // get <rnti, UeManager> map of connected UEs
    auto ueMap = m_rrc->GetUeMap();
    // gNB-wide PDCP volume in downlink
    uint64_t cellDlTxVolume = 0; // in kbit

    // sum of the per-user average latency
    double perUserAverageLatencySum = 0;

    std::unordered_map<uint64_t, std::string> uePmString{};

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        std::string ueImsiComplete = GetImsiString(imsi);

        uint64_t txDlPackets =
            m_e2PdcpStatsCalculator->GetDlTxPackets(imsi, 3); // LCID 3 is used for data
        uint64_t txBytes =
            std::round (m_e2PdcpStatsCalculator->GetDlTxData(imsi, 3) * 8 / 1e3); // in kbit, not byte
        cellDlTxVolume += txBytes;

        uint64_t txPdcpPduLteRlc = 0;
        uint64_t txPdcpPduBytesLteRlc = 0;
        auto drbMap = ue.second->GetDrbMap();
        NS_LOG_INFO("About to get into the DRB map for ue " << std::to_string(imsi) << " size " << drbMap.size());

        uint64_t txE2DlPduRlc = m_e2RlcStatsCalculator->GetDlTxPackets(imsi, 3);
        uint64_t txE2DlBytesRlc = std::round( m_e2RlcStatsCalculator->GetDlTxData(imsi, 3) * 8 / 1e3); // in kbit, not byte

        NS_LOG_INFO("ue id " << std::to_string(imsi) << " txE2DlPduRlc " << std::to_string(txE2DlPduRlc));
        NS_LOG_INFO("ue id " << std::to_string(imsi) << " txE2DlBytesRlc " << std::to_string(txE2DlBytesRlc));

        for (auto drb : drbMap)
        {
            txPdcpPduLteRlc += drb.second->m_rlc->GetTxPacketsInReportingPeriod();
            txPdcpPduBytesLteRlc += drb.second->m_rlc->GetTxBytesInReportingPeriod();
            NS_LOG_INFO("ue id " << std::to_string(imsi) << " txPdcpPduLteRlc " << std::to_string(txPdcpPduLteRlc));
            NS_LOG_INFO("ue id " << std::to_string(imsi) << " txPdcpPduBytesLteRlc " << std::to_string(txPdcpPduBytesLteRlc));
            drb.second->m_rlc->ResetRlcCounters();
        }

        txPdcpPduBytesLteRlc = std::round(txPdcpPduBytesLteRlc * 8 / 1e3); // in kbit, not byte
        NS_LOG_INFO("ue id " << std::to_string(imsi) <<" final txPdcpPduBytesLteRlc after *= 8 / 1e3: " << std::to_string(txPdcpPduBytesLteRlc));

        NS_LOG_INFO("txDlPackets (" << txDlPackets << ") vs txPdcpPduLteRlc ("
                                  << txPdcpPduLteRlc << ") and txBytes (" << txBytes << ") vs txPdcpPduBytesLteRlc (" << txPdcpPduBytesLteRlc
                                  << ")\n");

        /**
         * PDCP cell totals and LTE RLC counters can disagree: RLC buffers move
         * at handover, traffic can start before RLC attach, and a narrow LTE
         * carrier (e.g. 6 RB) often reports RLC bytes before PDCP stats catch up.
         * Clamp so the NR remainder stays non-negative instead of aborting.
         */
        if (txPdcpPduBytesLteRlc > txBytes || txPdcpPduLteRlc > txDlPackets)
        {
            NS_LOG_WARN("CU-UP LTE RLC exceeds PDCP (bytes "
                        << txPdcpPduBytesLteRlc << "/" << txBytes << " pkts "
                        << txPdcpPduLteRlc << "/" << txDlPackets << "); clamping");
            if (txPdcpPduBytesLteRlc > txBytes)
            {
                txPdcpPduBytesLteRlc = txBytes;
            }
            if (txPdcpPduLteRlc > txDlPackets)
            {
                txPdcpPduLteRlc = txDlPackets;
            }
        }

        uint64_t txPdcpPduNrRlc = std::max(static_cast<uint64_t>(0), txDlPackets - txPdcpPduLteRlc);
        double txPdcpPduBytesNrRlc = std::max(static_cast<uint64_t>(0), txBytes - txPdcpPduBytesLteRlc);

        NS_LOG_INFO("ue id " << std::to_string(imsi) 
                    << " According to LTE, txPdcpPduNrRlc " << std::to_string(txPdcpPduNrRlc) 
                    <<  " txPdcpPduBytesNrRlc " << std::to_string(txPdcpPduBytesNrRlc)
        );

        double pdcpLatency = m_e2PdcpStatsCalculator->GetDlDelay(imsi, 3) / 1e5; // unit: x 0.1 ms
        perUserAverageLatencySum += pdcpLatency;

        double pdcpThroughput = txBytes / m_e2Periodicity; // unit kbps

        NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                     << " " << std::to_string(m_cellId) << " cell, connected UE with IMSI " << std::to_string(imsi)
                     << " ueImsiString " << ueImsiComplete << " txDlPackets " << txDlPackets
                     << " txPdcpPduNrRlc " << txPdcpPduNrRlc 
                     << " txBytes " << txBytes
                     << " txPdcpPduLteRlc " << txPdcpPduLteRlc
                     << " txPdcpPduBytesLteRlc " << txPdcpPduBytesLteRlc
                     << " txPdcpPduBytesNrRlc " << txPdcpPduBytesNrRlc 
                     << " pdcpLatency " << pdcpLatency
                     << " pdcpThroughput " << pdcpThroughput);

        m_e2PdcpStatsCalculator->ResetResultsForImsiLcid(imsi, 3);

        if (ueHelper)
        {
            long numDrb = ue.second->GetDrbMap().size();
            ueHelper->AddeNBUePmItem(ueImsiComplete,
                                     txBytes,
                                     txDlPackets,
                                     pdcpThroughput,
                                     pdcpLatency,
                                     numDrb,
                                     0);
        }

        // We include here the NR RLC PDU and Bytes even though it is LTE since they traces are not working properly
        // To remove them from the report, remove last two elements keeping commas
        uePmString.insert(std::make_pair(
            imsi,
            std::to_string(txBytes) + "," + std::to_string(txDlPackets) + "," +
                std::to_string(pdcpThroughput) + "," + std::to_string(pdcpLatency) + "," + 
                std::to_string(txPdcpPduLteRlc) + "," + std::to_string(txPdcpPduBytesLteRlc)
                + "," + std::to_string(txPdcpPduBytesNrRlc)+ "," + std::to_string(txPdcpPduNrRlc)));
    }

    // get average cell latency
    double cellAverageLatency = 0;
    if (!ueMap.empty())
    {
        cellAverageLatency = perUserAverageLatencySum / ueMap.size();
    }

    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                 << " " << std::to_string(m_cellId) << " cell, connected UEs number "
                 << ueMap.size() << " cellAverageLatency " << cellAverageLatency);

    if (cellHelper)
    {
        cellHelper->AddeNBCellPmItem(m_cellId, cellAverageLatency, 0, cellDlTxVolume, ueMap.size());
    }

    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
                 << " " << std::to_string(m_cellId) << " cell volume Lte " << cellDlTxVolume);

    if (m_enableGymFileLogging && m_sendCuUp)
    {
        std::ofstream csv{};
        csv.open(m_cuUpFileName.c_str(), std::ios_base::app);
        if (!csv.is_open())
        {
            NS_FATAL_ERROR("Can't open file " << m_cuUpFileName.c_str());
        }

        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();

        // the string is timestamp,ueImsiComplete,DRB.PdcpSduDelayDl(cellAverageLatency),
        // m_pDCPBytesUL(0),m_pDCPBytesDL(cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID
        // (txBytes),Tot.PdcpSduNbrDl.UEID (txDlPackets), 
        // DRB.PdcpSduBitRateDl.UEID(pdcpThroughput),DRB.PdcpSduDelayDl.UEID(pdcpLatency), 
        // txPdcpPduLteRlc,txPdcpPduBytesLteRlc,
        // QosFlow.PdcpPduVolumeDL_Filter.UEID(txPdcpPduBytesNrRlc),
        // DRB.PdcpPduNbrDl.Qos.UEID(txPdcpPduNrRlc)

        // the last two are not available on LTE
        for (auto ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string ueImsiComplete = GetImsiString(imsi);

            auto uePms = uePmString.find(imsi)->second;

            std::string to_print = std::to_string(timestamp) + "," + ueImsiComplete + "," +
                                   std::to_string(cellAverageLatency) + "," + std::to_string(0) +
                                   "," + std::to_string(cellDlTxVolume) + "," + uePms + "\n"; 

            csv << to_print;
        }
        csv.close();
    }
    if (ueHelper)
    {
        m_e2UeIndication = ueHelper->CreateIndicationMessage("ue");
    }
    if (cellHelper)
    {
        m_e2CellIndication = cellHelper->CreateIndicationMessage("cell");
    }
    return nullptr;
}

void
LteEnbNetDevice::SaveActiveUes(void)
{
    NS_LOG_FUNCTION(this);
    m_ueRrcMean.push_back(m_rrc->GetUeMap().size());

    // We register this every 10 ms
    Simulator::Schedule(MilliSeconds(10), &LteEnbNetDevice::SaveActiveUes, this);
}

long
LteEnbNetDevice::ComputeMeanUes(void)
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
LteEnbNetDevice::BuildRicIndicationMessageCuCp(std::string plmId)
{
    (void)plmId;
    auto ueMap = m_rrc->GetUeMap();
    auto ueMapSize = ueMap.size();
    long meanRrcUes = ComputeMeanUes();

    std::unordered_map<uint64_t, std::string> uePmString{};

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        std::string ueImsiComplete = GetImsiString(imsi);
        long numDrb = ue.second->GetDrbMap().size();

        uePmString.insert(std::make_pair(imsi, std::to_string(numDrb) + "," + std::to_string(0)));
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

        // the string is timestamp, ueImsiComplete, numActiveUes, RRC.ConnMean,
        // DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0)

        uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();

        for (auto ue : ueMap)
        {
            uint64_t imsi = ue.second->GetImsi();
            std::string ueImsiComplete = GetImsiString(imsi);

            auto uePms = uePmString.find(imsi)->second;

            // SINR for the same cell
            ImsiCellIdPair_t cid{imsi, m_cellId};
            double sinrThisCell = 10 * std::log10(m_l3sinrMap[cid]);
            double convertedSinr = L3RrcMeasurements::ThreeGppMapSinr(sinrThisCell);

            std::string to_print =
                std::to_string(timestamp) + "," + ueImsiComplete + "," + std::to_string(ueMapSize) +
                "," + std::to_string(meanRrcUes)+ "," + uePms + "," + std::to_string(m_cellId) + "," +
                std::to_string(sinrThisCell) + "," + std::to_string(convertedSinr) + "\n";

            // NS_LOG_DEBUG(to_print);

            csv << to_print;
        }

        csv.close();
    }
    return nullptr;
}

void
LteEnbNetDevice::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
    (void)params;
    std::string plmId = "111";
    std::string gnbId = std::to_string(m_cellId);

    NS_LOG_DEBUG("LteEnbNetDevice " << std::to_string(m_cellId) << " BuildAndSendMessage at time "
                                    << Simulator::Now().GetSeconds());

    m_e2UeIndication = nullptr;
    m_e2CellIndication = nullptr;

    // [nsoran-qos] DU before CU-UP: CU-UP ResetResultsForImsiLcid would zero PDCP GetDlTxData.
    if (m_sendDu && m_enableGymFileLogging)
    {
        WriteDuReport();
    }

    if (m_sendCuUp || m_e2term)
    {
        BuildRicIndicationMessageCuUp(plmId);
    }
    if (m_sendCuCp)
    {
        BuildRicIndicationMessageCuCp(plmId);
    }

    if (m_e2term && m_e2Subscribed)
    {
        Ptr<KpmIndicationMessage> messages[2] = {m_e2UeIndication, m_e2CellIndication};
        for (Ptr<KpmIndicationMessage> msg : messages)
        {
            if (!msg || !msg->m_buffer)
            {
                continue;
            }
            Ptr<KpmIndicationHeader> header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
            if (!header)
            {
                continue;
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
        }
    }

    if (m_e2term && !m_enableGymFileLogging)
    {
        Simulator::ScheduleWithContext(1,
                                       Seconds(m_e2Periodicity),
                                       &LteEnbNetDevice::BuildAndSendReportMessage,
                                       this,
                                       m_e2ReportParams);
    }
    else
    {
        Simulator::Schedule(Seconds(m_e2Periodicity),
                            &LteEnbNetDevice::BuildAndSendReportMessage,
                            this,
                            m_e2ReportParams);
    }
}

void
LteEnbNetDevice::ConnectDuMacTrace()
{
    // [nsoran-qos] MAC grants (PRB/MCS) + PHY ndi (initial vs HARQ retx)
    if (m_duTraceConnected)
    {
        return;
    }
    Ptr<LteEnbMac> mac = GetMac();
    if (!mac)
    {
        NS_LOG_WARN("LTE DU: MAC not ready, cannot connect DlScheduling");
        return;
    }
    bool ok = mac->TraceConnectWithoutContext(
        "DlScheduling",
        MakeCallback(&LteEnbNetDevice::DlSchedulingForDu, this));
    NS_ABORT_MSG_IF(!ok, "Failed to connect LTE DlScheduling for DU PRB logging");
    Ptr<LteEnbPhy> phy = GetPhy();
    if (phy)
    {
        ok = phy->TraceConnectWithoutContext(
            "DlPhyTransmission",
            MakeCallback(&LteEnbNetDevice::DlPhyTransmissionForDu, this));
        NS_ABORT_MSG_IF(!ok, "Failed to connect LTE DlPhyTransmission for DU HARQ/SINR");
    }
    else
    {
        NS_LOG_WARN("LTE DU: PHY not ready, HARQ/SINR bins stay empty");
    }
    m_duTraceConnected = true;
}

void
LteEnbNetDevice::DlSchedulingForDu(DlSchedulingCallbackInfo info)
{
    LteDuUeStats& stats = m_duUeStats[info.rnti];
    stats.nTb++;
    stats.nPrbSum += info.nPrb;
    stats.tbBytes += info.sizeTb1 + info.sizeTb2;
    auto addMcs = [](LteDuUeStats& s, uint8_t mcs) {
        if (mcs <= 4)
        {
            s.mcs04++;
        }
        else if (mcs <= 9)
        {
            s.mcs59++;
        }
        else if (mcs <= 14)
        {
            s.mcs1014++;
        }
        else if (mcs <= 19)
        {
            s.mcs1519++;
        }
        else if (mcs <= 24)
        {
            s.mcs2024++;
        }
        else
        {
            s.mcs2529++;
        }
        if (mcs <= 9)
        {
            s.qpsk++;
        }
        else if (mcs <= 16)
        {
            s.qam16++;
        }
        else
        {
            s.qam64++;
        }
    };
    addMcs(stats, info.mcsTb1);
    if (info.sizeTb2 > 0)
    {
        addMcs(stats, info.mcsTb2);
    }
}

void
LteEnbNetDevice::DlPhyTransmissionForDu(PhyTransmissionStatParameters params)
{
    // [nsoran-qos] mmWave DU uses PHY rv==0 as initial; LTE uses ndi==1 (new TB).
    if (params.m_cellId != m_cellId)
    {
        return;
    }
    LteDuUeStats& stats = m_duUeStats[params.m_rnti];
    if (params.m_ndi == 1)
    {
        stats.nTbInitial++;
    }
    else
    {
        stats.nRetx++;
    }

    uint64_t imsi = params.m_imsi;
    if (imsi == 0 && m_rrc)
    {
        imsi = m_rrc->GetImsiFromRnti(params.m_rnti);
    }
    if (imsi == 0)
    {
        return;
    }
    ImsiCellIdPair_t cid{imsi, m_cellId};
    auto sinrIt = m_l3sinrMap.find(cid);
    if (sinrIt == m_l3sinrMap.end() || sinrIt->second <= 0)
    {
        return;
    }
    double sinrDb = 10.0 * std::log10(static_cast<double>(sinrIt->second));
    int bin = 6;
    if (sinrDb <= -6)
    {
        bin = 0;
    }
    else if (sinrDb <= 0)
    {
        bin = 1;
    }
    else if (sinrDb <= 6)
    {
        bin = 2;
    }
    else if (sinrDb <= 12)
    {
        bin = 3;
    }
    else if (sinrDb <= 18)
    {
        bin = 4;
    }
    else if (sinrDb <= 24)
    {
        bin = 5;
    }
    stats.sinrBin[bin]++;
}

uint32_t
LteEnbNetDevice::GetRlcBufferOccupancy(Ptr<LteRlc> rlc) const
{
    // [nsoran-qos] Same RLC peek as MmWaveEnbNetDevice::GetRlcBufferOccupancy
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
    return 0;
}

void
LteEnbNetDevice::WriteDuReport()
{
    // [nsoran-qos] Same CSV schema as mmWave du-cell-*.txt.
    // RRU.PrbUsedDl.UEID = sum(nPrb) / nTtis (average PRBs per LTE TTI).
    // Gym EE later: occupancy = LTE_value/dlAvailablePrbs + NR_value/139 (do not add raw).
    if (m_duFileName.empty())
    {
        return;
    }

    std::ofstream csv;
    csv.open(m_duFileName.c_str(), std::ios_base::app);
    if (!csv.is_open())
    {
        NS_LOG_ERROR("Can't open LTE DU file " << m_duFileName);
        return;
    }

    auto ueMap = m_rrc->GetUeMap();
    uint32_t nTtis = static_cast<uint32_t>(std::round(m_e2Periodicity * 1000.0));
    if (nTtis == 0)
    {
        nTtis = 1;
    }
    long dlAvailablePrbs = GetDlBandwidth();
    if (dlAvailablePrbs <= 0)
    {
        dlAvailablePrbs = 100;
    }

    uint32_t macPduCell = 0;
    uint32_t macPduInitialCell = 0;
    uint32_t macRetxCell = 0;
    uint32_t qpskCell = 0;
    uint32_t qam16Cell = 0;
    uint32_t qam64Cell = 0;
    uint32_t mcs04Cell = 0;
    uint32_t mcs59Cell = 0;
    uint32_t mcs1014Cell = 0;
    uint32_t mcs1519Cell = 0;
    uint32_t mcs2024Cell = 0;
    uint32_t mcs2529Cell = 0;
    uint32_t sinrBinCell[7]{};
    uint32_t rlcBufferCell = 0;
    uint64_t volumeCell = 0;
    double prbCell = 0.0;

    std::unordered_map<uint64_t, std::string> uePmStringDu;
    for (auto ue : ueMap)
    {
        uint16_t rnti = ue.first;
        uint64_t imsi = ue.second->GetImsi();
        LteDuUeStats stats{};
        auto it = m_duUeStats.find(rnti);
        if (it != m_duUeStats.end())
        {
            stats = it->second;
        }
        if (stats.nTbInitial == 0 && stats.nRetx == 0)
        {
            stats.nTbInitial = stats.nTb;
        }

        double macPrb = static_cast<double>(stats.nPrbSum) / static_cast<double>(nTtis);
        uint64_t volumeKbit = static_cast<uint64_t>(std::round(stats.tbBytes * 8.0 / 1e3));
        double thrKbps = 0.0;
        double thrPdcpKbps = 0.0;
        if (m_e2Periodicity > 0)
        {
            // LTE air bits this period (MAC TB), not the combined EN-DC PDCP offered load.
            thrKbps = stats.tbBytes * 8.0 / 1e3 / m_e2Periodicity;
        }
        if (m_e2PdcpStatsCalculator && m_e2Periodicity > 0)
        {
            double txBytes = m_e2PdcpStatsCalculator->GetDlTxData(imsi, 3);
            thrPdcpKbps = txBytes * 8.0 / 1e3 / m_e2Periodicity;
        }

        uint32_t rlcBufferOccup = 0;
        auto drbMap = ue.second->GetDrbMap();
        for (auto drb : drbMap)
        {
            rlcBufferOccup += GetRlcBufferOccupancy(drb.second->m_rlc);
        }
        auto rlcMap = ue.second->GetRlcMap();
        for (auto drb : rlcMap)
        {
            rlcBufferOccup += GetRlcBufferOccupancy(drb.second->m_rlc);
        }

        bool anySinr = false;
        for (int b = 0; b < 7; ++b)
        {
            if (stats.sinrBin[b] > 0)
            {
                anySinr = true;
                break;
            }
        }
        if (!anySinr)
        {
            ImsiCellIdPair_t cid{imsi, m_cellId};
            auto sinrIt = m_l3sinrMap.find(cid);
            if (sinrIt != m_l3sinrMap.end() && sinrIt->second > 0)
            {
                double sinrDb = 10.0 * std::log10(static_cast<double>(sinrIt->second));
                int bin = 6;
                if (sinrDb <= -6)
                {
                    bin = 0;
                }
                else if (sinrDb <= 0)
                {
                    bin = 1;
                }
                else if (sinrDb <= 6)
                {
                    bin = 2;
                }
                else if (sinrDb <= 12)
                {
                    bin = 3;
                }
                else if (sinrDb <= 18)
                {
                    bin = 4;
                }
                else if (sinrDb <= 24)
                {
                    bin = 5;
                }
                uint32_t n = stats.nTb > 0 ? stats.nTb : 1;
                stats.sinrBin[bin] += n;
            }
        }

        macPduCell += stats.nTb;
        macPduInitialCell += stats.nTbInitial;
        macRetxCell += stats.nRetx;
        qpskCell += stats.qpsk;
        qam16Cell += stats.qam16;
        qam64Cell += stats.qam64;
        mcs04Cell += stats.mcs04;
        mcs59Cell += stats.mcs59;
        mcs1014Cell += stats.mcs1014;
        mcs1519Cell += stats.mcs1519;
        mcs2024Cell += stats.mcs2024;
        mcs2529Cell += stats.mcs2529;
        volumeCell += volumeKbit;
        prbCell += macPrb;
        rlcBufferCell += rlcBufferOccup;
        for (int b = 0; b < 7; ++b)
        {
            sinrBinCell[b] += stats.sinrBin[b];
        }

        uePmStringDu[imsi] =
            std::to_string(stats.nTb) + "," + std::to_string(stats.nTbInitial) + "," +
            std::to_string(stats.qpsk) + "," + std::to_string(stats.qam16) + "," +
            std::to_string(stats.qam64) + "," + std::to_string(stats.nRetx) + "," +
            std::to_string(volumeKbit) + "," + std::to_string(macPrb) + "," +
            std::to_string(stats.mcs04) + "," + std::to_string(stats.mcs59) + "," +
            std::to_string(stats.mcs1014) + "," + std::to_string(stats.mcs1519) + "," +
            std::to_string(stats.mcs2024) + "," + std::to_string(stats.mcs2529) + "," +
            std::to_string(stats.sinrBin[0]) + "," + std::to_string(stats.sinrBin[1]) + "," +
            std::to_string(stats.sinrBin[2]) + "," + std::to_string(stats.sinrBin[3]) + "," +
            std::to_string(stats.sinrBin[4]) + "," + std::to_string(stats.sinrBin[5]) + "," +
            std::to_string(stats.sinrBin[6]) + "," + std::to_string(rlcBufferOccup) + "," +
            std::to_string(thrKbps) + "," + std::to_string(thrPdcpKbps);
    }

    long dlPrbUsage =
        std::min(static_cast<long>(prbCell / dlAvailablePrbs * 100.0), static_cast<long>(100));
    std::string plmId = "111";
    uint64_t timestamp = m_startTime + Simulator::Now().GetMilliSeconds();

    std::string to_print_cell =
        plmId + "," + std::to_string(m_cellId) + "," + std::to_string(dlAvailablePrbs) + "," +
        std::to_string(dlAvailablePrbs) + ",1," + std::to_string(dlPrbUsage) + ",0," +
        std::to_string(macPduCell) + "," + std::to_string(macPduInitialCell) + "," +
        std::to_string(qpskCell) + "," + std::to_string(qam16Cell) + "," +
        std::to_string(qam64Cell) + "," + std::to_string(static_cast<long>(std::ceil(prbCell))) +
        "," + std::to_string(macRetxCell) + "," + std::to_string(volumeCell) + "," +
        std::to_string(mcs04Cell) + "," + std::to_string(mcs59Cell) + "," +
        std::to_string(mcs1014Cell) + "," + std::to_string(mcs1519Cell) + "," +
        std::to_string(mcs2024Cell) + "," + std::to_string(mcs2529Cell) + "," +
        std::to_string(sinrBinCell[0]) + "," + std::to_string(sinrBinCell[1]) + "," +
        std::to_string(sinrBinCell[2]) + "," + std::to_string(sinrBinCell[3]) + "," +
        std::to_string(sinrBinCell[4]) + "," + std::to_string(sinrBinCell[5]) + "," +
        std::to_string(sinrBinCell[6]) + "," + std::to_string(rlcBufferCell) + "," +
        std::to_string(ueMap.size());

    for (auto ue : ueMap)
    {
        uint64_t imsi = ue.second->GetImsi();
        auto pmIt = uePmStringDu.find(imsi);
        if (pmIt == uePmStringDu.end())
        {
            continue;
        }
        csv << timestamp << "," << GetImsiString(imsi) << "," << to_print_cell << "," << pmIt->second
            << "\n";
    }
    csv.close();
    m_duUeStats.clear();
}

void
LteEnbNetDevice::SetStartTime(uint64_t st)
{
    m_startTime = st;
}


uint64_t
LteEnbNetDevice::GetStartTime()
{
    return m_startTime;
}

double
LteEnbNetDevice::GetE2Periodicity ()
{
    return m_e2Periodicity;
}

} // namespace ns3
