/*
 * VanetHeader.cc
 *
 *  Created on: Sep 26, 2011
 *      Author: andrer
 */

#include "VanetHeader.h"

using namespace ns3;

VanetHeader::VanetHeader()
{
}

VanetHeader::~VanetHeader()
{
}

TypeId VanetHeader::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::VanetHeader")
	.SetParent<Header> ()
	.AddConstructor<VanetHeader> ()
	;
	return tid;
}

TypeId
VanetHeader::GetInstanceTypeId (void) const
{
	return GetTypeId ();
}

void
VanetHeader::Print (std::ostream &os) const
{
	// This method is invoked by the packet printing
	// routines to print the content of my header.
	os << "src=" << m_src << std::endl;
	os << "id=" << m_id << std::endl;
}
uint32_t
VanetHeader::GetSerializedSize (void) const
{
	// we reserve 4 bytes for our header.
	return 4;
}

void
VanetHeader::Serialize (Buffer::Iterator start) const
{
	// we can serialize four bytes at the start of the buffer.
	// we write them in network byte order.
	start.WriteHtonU16 (m_src);
	start.WriteHtonU16 (m_id);
}
uint32_t
VanetHeader::Deserialize (Buffer::Iterator start)
{
	// we can deserialize four bytes from the start of the buffer.
	// we read them in network byte order and store them
	// in host byte order.
	m_src = start.ReadNtohU16 ();
	m_id = start.ReadNtohU16 ();

	// we return the number of bytes effectively read.
	return 4;
}

void VanetHeader::SetSource(int src)
{
	m_src = (int16_t)src;
}

void VanetHeader::SetID(unsigned int id)
{
	m_id = (uint16_t)id;
}

int VanetHeader::GetSource(void) const
{
	return (int)m_src;
}

unsigned int VanetHeader::GetID(void) const
{
	return (unsigned int)m_id;
}
