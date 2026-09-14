/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
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
 */

#include "mmwave-mac-scheduler.h"

#include <ns3/log.h>
#include <ns3/uinteger.h>

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWaveMacScheduler");

namespace mmwave
{

NS_OBJECT_ENSURE_REGISTERED(MmWaveMacScheduler);

MmWaveMacScheduler::MmWaveMacScheduler()
    : m_asmMode(1)
{
    NS_LOG_FUNCTION(this);
}

MmWaveMacScheduler::~MmWaveMacScheduler()
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveMacScheduler::DoDispose(void)
{
    NS_LOG_FUNCTION(this);
}

TypeId
MmWaveMacScheduler::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::MmWaveMacScheduler")
            .SetParent<Object>()
            .AddAttribute("AsmMode",
                          "ASM data duty: 1=100%, 2=75%, 3=50%, 4=25%, 5=0% of data "
                          "symbols. 6=TurnOff (HO barred, evict to other mmWave or LTE).",
                          UintegerValue(1),
                          MakeUintegerAccessor(&MmWaveMacScheduler::SetAsmMode,
                                               &MmWaveMacScheduler::GetAsmMode),
                          MakeUintegerChecker<uint8_t>(1, 6));

    return tid;
}

void
MmWaveMacScheduler::SetAsmMode(uint8_t mode)
{
    NS_ABORT_MSG_IF(mode < 1 || mode > 6, "ASM mode must be 1..6");
    m_asmMode = mode;
}

uint8_t
MmWaveMacScheduler::GetAsmMode() const
{
    return m_asmMode;
}

double
MmWaveMacScheduler::AsmDutyCycle(uint8_t mode)
{
    NS_ABORT_MSG_IF(mode < 1 || mode > 6, "ASM mode must be 1..6");
    if (mode == 6)
    {
        return 0.0;
    }
    return (5 - mode) * 0.25;
}

int
MmWaveMacScheduler::GetAvailableDataSymbols() const
{
    NS_ASSERT(m_phyMacConfig);
    int full = static_cast<int>(m_phyMacConfig->GetSymbPerSlot()) -
               static_cast<int>(m_phyMacConfig->GetDlCtrlSymbols()) -
               static_cast<int>(m_phyMacConfig->GetUlCtrlSymbols());
    if (full < 0)
    {
        full = 0;
    }
    return static_cast<int>(std::floor(full * AsmDutyCycle(m_asmMode) + 1e-9));
}

/*void
MmWaveMacScheduler::ConfigureCommonParameters (Ptr<MmWavePhyMacCommon> config)
{
        m_phyMacConfig = config;
}*/

} // namespace mmwave

} // namespace ns3
