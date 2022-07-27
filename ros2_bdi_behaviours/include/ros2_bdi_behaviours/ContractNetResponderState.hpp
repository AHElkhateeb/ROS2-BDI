#pragma once
#include "ros2_bdi_behaviours/ContractNetResponder.hpp"

// Forward declaration to resolve circular dependency/include
class ContractNetResponder;
struct MsgReceived;
struct ActionOver;

class ContractNetResponderState
{
public:
	virtual void entry(ContractNetResponder* contractNetResponder) = 0;
	virtual void react(ContractNetResponder* contractNetResponder, MsgReceived const & Message) = 0;
	virtual void react(ContractNetResponder* contractNetResponder, ActionOver const & Message) = 0;
	virtual void exit(ContractNetResponder* contractNetResponder) = 0;
	virtual ~ContractNetResponderState() {}
};