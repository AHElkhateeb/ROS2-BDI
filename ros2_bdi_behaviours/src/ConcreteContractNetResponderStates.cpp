#include "ros2_bdi_behaviours/ConcreteContractNetResponderStates.hpp"

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(ContractNetResponder* contractNetResponder){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message){}
void EndProtocol::react(ContractNetResponder* contractNetResponder, ActionOver const & Message){}

void EndProtocol::entry(ContractNetResponder* contractNetResponder)
{
	//deletes ConvID SharedPointer
  auto convID = std_msgs::msg::String().set__data(contractNetResponder->cfp.getConversationId());
  contractNetResponder->del_conv_client_publisher_->publish(convID);
}

// ----------------------------------------------------------------------------
// State: PerformAction
//
void PerformAction::exit(ContractNetResponder* contractNetResponder){}

void PerformAction::entry(ContractNetResponder* contractNetResponder)
{
  contractNetResponder->inform = contractNetResponder->handleAcceptProposal(contractNetResponder->cfp, contractNetResponder->propose, contractNetResponder->accept);
	//send the output of the handleAcceptProposal function
  contractNetResponder->react(ActionOver());
}

void PerformAction::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "CANCEL")
    {
      std::cout << "PerformAction state: CANCEL message received" << std::endl;
      contractNetResponder->setState(EndProtocol::getInstance());
    }
    else
    {
      std::cout << "PerformAction state: Out of sequence message received" << std::endl;
      //send a NOT-UNDERSTOOD msg to communicate communication problems.
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
	//send the output of the handleCfp function
}

void CfpEvaluation::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "ACCEPT")
      {
        std::cout << "CfpEvaluation state: ACCEPT message received" << std::endl;
        contractNetResponder->accept = Message.msg;
        contractNetResponder->setState(PerformAction::getInstance());
      }
    else if (Message.msg.getPerformative() == "REJECT")
      {
        std::cout << "CfpEvaluation state: REJECT message received" << std::endl;
        contractNetResponder->reject = Message.msg;
        contractNetResponder->handleRejectProposal(contractNetResponder->cfp, contractNetResponder->propose, contractNetResponder->reject);
        contractNetResponder->setState(EndProtocol::getInstance());
      }
      else
      	{
          std::cout << "CfpEvaluation state: Out of sequence message received" << std::endl;
          //send a NOT-UNDERSTOOD msg to communicate communication problems.
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

void ReceiveCfp::react(ContractNetResponder* contractNetResponder, MsgReceived const & Message)
{
	if (Message.msg.getPerformative() == "CFP")
      {
        contractNetResponder->cfp = Message.msg;
        std::cout << "ReceiveCfp state: CFP message received" << std::endl;
        contractNetResponder->setState(CfpEvaluation::getInstance());
      }
    else
      {
        std::cout << "ReceiveCfp state: Out of sequence message received" << std::endl;
        //send a NOT-UNDERSTOOD msg to communicate communication problems.
      }
}