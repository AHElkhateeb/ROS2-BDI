#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"

// Forward declaration to resolve circular dependency/include
class ContractNetInitiator;
struct MsgReceived;
struct ActionOver;

class ContractNetInitiatorState
{
public:
	virtual void entry(ContractNetInitiator* contractNetInitiator) = 0;
	virtual void react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message) = 0;
	virtual void exit(ContractNetInitiator* contractNetInitiator) = 0;
	virtual ~ContractNetInitiatorState() {}
};