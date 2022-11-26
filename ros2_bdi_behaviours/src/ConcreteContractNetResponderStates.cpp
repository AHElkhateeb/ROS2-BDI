#include "ros2_bdi_behaviours/ConcreteContractNetResponderStates.hpp"

using namespace ResponderStates;

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(ContractNetResponder* contractNetResponder){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, ACLMessage const & Message){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, ActionOver const & Message){}

void EndProtocol::entry(ContractNetResponder* contractNetResponder)
{
	//deletes ConvID SharedPointer
  contractNetResponder->deleteConvID();
}

// ----------------------------------------------------------------------------
// State: PerformAction
//
void PerformAction::exit(ContractNetResponder* contractNetResponder){}

void PerformAction::entry(ContractNetResponder* contractNetResponder)
{
  contractNetResponder->inform = contractNetResponder->handleAcceptProposal(contractNetResponder->cfp, contractNetResponder->propose, contractNetResponder->accept);
	contractNetResponder->sendMsg(contractNetResponder->inform);
  contractNetResponder->react(ActionOver());
}

void PerformAction::react(ContractNetResponder* contractNetResponder, ACLMessage const & Message)
{
	if (Message.getPerformative() == AclMsg::CANCEL)
    {
      std::cout << "PerformAction state: CANCEL message received" << std::endl;
      contractNetResponder->setState(EndProtocol::getInstance());
    }
    else
    {
      std::cout << "PerformAction state: Out of sequence message received" << std::endl;
      contractNetResponder->sendMsg(Message.createReply()); //Sends back a NOTUNDERSTOOD Message
      contractNetResponder->setState(EndProtocol::getInstance());
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
	contractNetResponder->propose = contractNetResponder->handleCfp(contractNetResponder->cfp);
	contractNetResponder->sendMsg(contractNetResponder->propose);
  if(contractNetResponder->propose.getPerformative() != AclMsg::PROPOSE) 
  {
    contractNetResponder->setState(EndProtocol::getInstance());
  }
}

void CfpEvaluation::react(ContractNetResponder* contractNetResponder, ACLMessage const & Message)
{
	if (Message.getPerformative() == AclMsg::ACCEPT_PROPOSAL)
      {
        std::cout << "CfpEvaluation state: ACCEPT message received" << std::endl;
        contractNetResponder->accept = Message;
        contractNetResponder->setState(PerformAction::getInstance());
      }
    else if (Message.getPerformative() == AclMsg::REJECT_PROPOSAL)
      {
        std::cout << "CfpEvaluation state: REJECT message received" << std::endl;
        contractNetResponder->reject = Message;
        contractNetResponder->handleRejectProposal(contractNetResponder->cfp, contractNetResponder->propose, contractNetResponder->reject);
        contractNetResponder->setState(EndProtocol::getInstance());
      }
      else
      {
        std::cout << "CfpEvaluation state: Out of sequence message received" << std::endl;
        contractNetResponder->sendMsg(Message.createReply()); //Sends back a NOTUNDERSTOOD Message
        contractNetResponder->setState(EndProtocol::getInstance());
      }
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

void ReceiveCfp::react(ContractNetResponder* contractNetResponder, ACLMessage const & Message)
{
	if (Message.getPerformative() == AclMsg::CALL_FOR_PROPOSAL)
      {
        contractNetResponder->cfp = Message;
        std::cout << "ReceiveCfp state: CFP message received" << std::endl;
        contractNetResponder->setState(CfpEvaluation::getInstance());
      }
    else
      {
        std::cout << "ReceiveCfp state: Out of sequence message received" << std::endl;
        contractNetResponder->sendMsg(Message.createReply()); //Sends back a NOTUNDERSTOOD Message
        contractNetResponder->setState(EndProtocol::getInstance());
      }
}