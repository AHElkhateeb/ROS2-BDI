#pragma once
#include "ros2_bdi_behaviours/ContractNetInitiatorState.hpp"
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"

namespace InitiatorStates
{
	// ----------------------------------------------------------------------------
	// State: EndProtocol
	//
	class EndProtocol : public ContractNetInitiatorState
	{
	public:
		void entry(ContractNetInitiator* contractNetInitiator);
		void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message);
		void exit(ContractNetInitiator* contractNetInitiator);
		static ContractNetInitiatorState& getInstance(){ static EndProtocol singleton; return singleton; }

	private:
		EndProtocol() {}									//Hide the constructor
		EndProtocol(const EndProtocol& other);				//Hide the copy constructor
		EndProtocol& operator=(const EndProtocol& other);	//Hide the assignment operator
	};

	// ----------------------------------------------------------------------------
	// State: WaitForResult
	//
	class WaitForResult : public ContractNetInitiatorState
	{
	public:
		void entry(ContractNetInitiator* contractNetInitiator);
		void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message);
		void exit(ContractNetInitiator* contractNetInitiator);
		static ContractNetInitiatorState& getInstance(){ static WaitForResult singleton; return singleton; }

	private:
		WaitForResult() {}										//Hide the constructor
		WaitForResult(const WaitForResult& other);				//Hide the copy constructor
		WaitForResult& operator=(const WaitForResult& other);	//Hide the assignment operator
	};

	// ----------------------------------------------------------------------------
	// State: EvaluateBids
	//
	class EvaluateBids : public ContractNetInitiatorState
	{
	public:
		void entry(ContractNetInitiator* contractNetInitiator);
		void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message);
		void exit(ContractNetInitiator* contractNetInitiator);
		static ContractNetInitiatorState& getInstance(){ static EvaluateBids singleton; return singleton; }

	private:
		EvaluateBids() {}										//Hide the constructor
		EvaluateBids(const EvaluateBids& other);				//Hide the copy constructor
		EvaluateBids& operator=(const EvaluateBids& other);	//Hide the assignment operator
	};

	// ----------------------------------------------------------------------------
	// State: StoreBids
	//
	class StoreBids : public ContractNetInitiatorState
	{
	public:
		void entry(ContractNetInitiator* contractNetInitiator);
		void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message);
		void exit(ContractNetInitiator* contractNetInitiator);
		static ContractNetInitiatorState& getInstance(){ static StoreBids singleton; return singleton; }

	private:
		StoreBids() {}										//Hide the constructor
		StoreBids(const StoreBids& other);				//Hide the copy constructor
		StoreBids& operator=(const StoreBids& other);	//Hide the assignment operator
	};

	// ----------------------------------------------------------------------------
	// State: SendCfp
	//
	class SendCfp : public ContractNetInitiatorState
	{
	public:
		void entry(ContractNetInitiator* contractNetInitiator);
		void react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message);
		void exit(ContractNetInitiator* contractNetInitiator);
		static ContractNetInitiatorState& getInstance(){ static SendCfp singleton; return singleton; }

	private:
		SendCfp() {}									//Hide the constructor
		SendCfp(const SendCfp& other);			//Hide the copy constructor
		SendCfp& operator=(const SendCfp& other);	//Hide the assignment operator
	};
}