#pragma once
#include "ros2_bdi_behaviours/ContractNetResponderState.hpp"
#include "ros2_bdi_behaviours/ContractNetResponder.hpp"

// ----------------------------------------------------------------------------
// State: EndProtocol
//
class EndProtocol : public ContractNetResponderState
{
public:
	void entry(ContractNetResponder* contractNetResponder);
	void react(ContractNetResponder* contractNetResponder, ACLMessage const & Message);
	void react(ContractNetResponder* contractNetResponder, ActionOver const & Message);
	void exit(ContractNetResponder* contractNetResponder);
	static ContractNetResponderState& getInstance(){ static EndProtocol singleton; return singleton; }

private:
	EndProtocol() {}									//Hide the constructor
	EndProtocol(const EndProtocol& other);				//Hide the copy constructor
	EndProtocol& operator=(const EndProtocol& other);	//Hide the assignment operator
};

// ----------------------------------------------------------------------------
// State: PerformAction
//
class PerformAction : public ContractNetResponderState
{
public:
	void entry(ContractNetResponder* contractNetResponder);
	void react(ContractNetResponder* contractNetResponder, ACLMessage const & Message);
	void react(ContractNetResponder* contractNetResponder, ActionOver const & Message);
	void exit(ContractNetResponder* contractNetResponder);
	static ContractNetResponderState& getInstance(){ static PerformAction singleton; return singleton; }

private:
	PerformAction() {}										//Hide the constructor
	PerformAction(const PerformAction& other);				//Hide the copy constructor
	PerformAction& operator=(const PerformAction& other);	//Hide the assignment operator
};

// ----------------------------------------------------------------------------
// State: CfpEvaluation
//
class CfpEvaluation : public ContractNetResponderState
{
public:
	void entry(ContractNetResponder* contractNetResponder);
	void react(ContractNetResponder* contractNetResponder, ACLMessage const & Message);
	void react(ContractNetResponder* contractNetResponder, ActionOver const & Message);
	void exit(ContractNetResponder* contractNetResponder);
	static ContractNetResponderState& getInstance(){ static CfpEvaluation singleton; return singleton; }

private:
	CfpEvaluation() {}										//Hide the constructor
	CfpEvaluation(const CfpEvaluation& other);				//Hide the copy constructor
	CfpEvaluation& operator=(const CfpEvaluation& other);	//Hide the assignment operator
};

// ----------------------------------------------------------------------------
// State: ReceiveCfp
//
class ReceiveCfp : public ContractNetResponderState
{
public:
	void entry(ContractNetResponder* contractNetResponder);
	void react(ContractNetResponder* contractNetResponder, ACLMessage const & Message);
	void react(ContractNetResponder* contractNetResponder, ActionOver const & Message);
	void exit(ContractNetResponder* contractNetResponder);
	static ContractNetResponderState& getInstance(){ static ReceiveCfp singleton; return singleton; }

private:
	ReceiveCfp() {}									//Hide the constructor
	ReceiveCfp(const ReceiveCfp& other);			//Hide the copy constructor
	ReceiveCfp& operator=(const ReceiveCfp& other);	//Hide the assignment operator
};