#include "ros2_bdi_behaviours/ConcreteContractNetResponderStates.hpp"

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(ContractNetResponder* contractNetResponder){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, ActionOver const & Message){}

void EndProtocol::entry(ContractNetResponder* contractNetResponder)
{
	//deleteConvIDSharedPointer();
}

// ----------------------------------------------------------------------------
// State: PerformAction
//
void PerformAction::exit(ContractNetResponder* contractNetResponder){}

void PerformAction::entry(ContractNetResponder* contractNetResponder)
{
    contractNetResponder->handleAcceptProposal(contractNetResponder->last_message);
	//send the output of the handleAcceptProposal function
    contractNetResponder->react(ActionOver());
}

void PerformAction::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "CANCEL")
    {
      std::cout << "PerformAction state: CANCEL message received" << std::endl;
      contractNetResponder->last_message = Message.msg;
      contractNetResponder->setState(EndProtocol::getInstance());
    }
    else
    {
      std::cout << "PerformAction state: Out of sequence message received" << std::endl;
    }
}

void PerformAction::react(ContractNetResponder* contractNetResponder, ActionOver const & Message)
{
	contractNetResponder->setState(EndProtocol::getInstance());
}

// ----------------------------------------------------------------------------
// State: CfpEvaluation
//
void CfpEvaluation::exit(ContractNetResponder* contractNetResponder){}
void CfpEvaluation::react(ContractNetResponder* contractNetResponder, ActionOver const & Message){}

void CfpEvaluation::entry(ContractNetResponder* contractNetResponder)
{
	contractNetResponder->handleCfp(contractNetResponder->last_message);
	//send the output of the handleCfp function
}

void CfpEvaluation::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "ACCEPT")
      {
        std::cout << "CfpEvaluation state: ACCEPT message received" << std::endl;
        contractNetResponder->last_message = Message.msg;
        contractNetResponder->setState(PerformAction::getInstance());
      }
    else if (Message.msg.getPerformative() == "REJECT")
      {
        std::cout << "CfpEvaluation state: REJECT message received" << std::endl;
        contractNetResponder->last_message = Message.msg;
        contractNetResponder->setState(EndProtocol::getInstance());
      }
      else
      	std::cout << "CfpEvaluation state: Out of sequence message received" << std::endl;
}


// ----------------------------------------------------------------------------
// State: ReceiveCfp
//
void ReceiveCfp::exit(ContractNetResponder* contractNetResponder){}
void ReceiveCfp::react(ContractNetResponder* contractNetResponder, ActionOver const & Message){}

void ReceiveCfp::entry(ContractNetResponder* contractNetResponder)
{
	std::cout << "Waiting for CFP" << std::endl;
}

void ReceiveCfp::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "CFP")
      {
        std::cout << "ReceiveCfp state: CFP message received" << std::endl;
        contractNetResponder->last_message = Message.msg;
        contractNetResponder->setState(CfpEvaluation::getInstance());
      }
    else
      std::cout << "ReceiveCfp state: Out of sequence message received" << std::endl;
}