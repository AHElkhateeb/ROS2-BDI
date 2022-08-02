#include "ros2_bdi_behaviours/ConcreteContractNetInitiatorStates.hpp"

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(ContractNetInitiator* contractNetInitiator){}
void EndProtocol::react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message){}
void EndProtocol::entry(ContractNetInitiator* contractNetInitiator)
{
  //deletes ConvID SharedPointer
  auto convID = std_msgs::msg::String().set__data(contractNetInitiator->cfp.getConversationId());
  contractNetInitiator->del_conv_client_publisher_->publish(convID);
}

// ----------------------------------------------------------------------------
// State: WaitForResult
//
void WaitForResult::exit(ContractNetInitiator* contractNetInitiator){}
void WaitForResult::entry(ContractNetInitiator* contractNetInitiator)
{
  std::cout << "WaitForResult state: Waiting for result " << std::endl;
}

void WaitForResult::react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message)
{
  if(Message.msg.getPerformative()=="INFORM")
  {
    contractNetInitiator->informs.push_back(Message.msg);
    contractNetInitiator->handleInform(Message.msg);
  }
  else if(Message.msg.getPerformative()=="FAILURE")
  {
    contractNetInitiator->informs.push_back(Message.msg);
    contractNetInitiator->handleFailure(Message.msg);
  }
  else
  {
    contractNetInitiator->informs.push_back(Message.msg);
    std::cout << "StoreBids state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message.msg);
    //send a NOT-UNDERSTOOD msg to communicate communication problems.
  }

  if(contractNetInitiator->informs.size() == contractNetInitiator->nAcceptances)
  {
    contractNetInitiator->handleAllResultNotifications(contractNetInitiator->informs);
    contractNetInitiator->setState(EndProtocol::getInstance());
  }
}
// ----------------------------------------------------------------------------
// State: EvaluateBids
//
void EvaluateBids::react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message){}
void EvaluateBids::exit(ContractNetInitiator* contractNetInitiator){}
void EvaluateBids::entry(ContractNetInitiator* contractNetInitiator)
{
  contractNetInitiator->acceptances = contractNetInitiator->handleAllResponses(contractNetInitiator->responses);
  //send(contractNetInitiator->acceptances)
  for(unsigned int i = 0; i < contractNetInitiator->acceptances.size(); i++)
  {
    if(contractNetInitiator->acceptances[i].getPerformative()=="ACCEPT")
      contractNetInitiator->nAcceptances++;
  }
  
  if(contractNetInitiator->nAcceptances > 0)
  {
    contractNetInitiator->setState(WaitForResult::getInstance());
  }
  else
  {
    contractNetInitiator->handleAllRejected();
    contractNetInitiator->setState(EndProtocol::getInstance());
  }
}

// ----------------------------------------------------------------------------
// State: StoreBids
//
void StoreBids::entry(ContractNetInitiator* contractNetInitiator){}
void StoreBids::exit(ContractNetInitiator* contractNetInitiator){}

void StoreBids::react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message)
{
  if(Message.msg.getPerformative()=="PROPOSE")
  {
    contractNetInitiator->responses.push_back(Message.msg);
    contractNetInitiator->handlePropose(Message.msg);
  }
  else if(Message.msg.getPerformative()=="REJECT")
  {
    contractNetInitiator->responses.push_back(Message.msg);
    contractNetInitiator->handleReject(Message.msg);
  }
  else
  {
    contractNetInitiator->responses.push_back(Message.msg);
    std::cout << "StoreBids state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message.msg);
    //send a NOT-UNDERSTOOD msg to communicate communication problems.
  }

  if(contractNetInitiator->responses.size() == contractNetInitiator->nResponders)
    contractNetInitiator->setState(EvaluateBids::getInstance());
}

// ----------------------------------------------------------------------------
// State: SendCfp
//
void SendCfp::entry(ContractNetInitiator* contractNetInitiator){}
void SendCfp::exit(ContractNetInitiator* contractNetInitiator){}
void SendCfp::react(ContractNetInitiator* contractNetInitiator, MsgReceived const & Message)
{
  if (Message.msg.getPerformative() == "CFP") //TO-DO: && Message.msg.getSender() == "this agent identifier" change it with this agent's ID parameter
    {
      contractNetInitiator->cfp = Message.msg;
      contractNetInitiator->nResponders = Message.msg.getReceivers().size();
      std::cout << "SendCfp state: CFP message to be sent to " << contractNetInitiator->nResponders << " agents" << std::endl;
      //send(CFP)
      contractNetInitiator->setState(StoreBids::getInstance());
    }
  else
    {
      std::cout << "SendCfp state: Message passed to this behaviour should be CFP and AID == this agent's AID" << std::endl;
      //send a NOT-UNDERSTOOD msg to communicate communication problems.
    }

}