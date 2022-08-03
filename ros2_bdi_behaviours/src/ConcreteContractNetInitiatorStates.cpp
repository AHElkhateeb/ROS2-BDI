#include "ros2_bdi_behaviours/ConcreteContractNetInitiatorStates.hpp"

using namespace InitiatorStates;

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(ContractNetInitiator* contractNetInitiator){ std::cout << "Exiting EndProtocol state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl; }
void EndProtocol::react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message){}
void EndProtocol::entry(ContractNetInitiator* contractNetInitiator)
{
  std::cout << "Entering EndProtocol state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;

  //deletes ConvID SharedPointer
  auto convID = std_msgs::msg::String().set__data(contractNetInitiator->cfp.getConversationId());
  contractNetInitiator->del_conv_client_publisher_->publish(convID);
}

// ----------------------------------------------------------------------------
// State: WaitForResult
//
void WaitForResult::exit(ContractNetInitiator* contractNetInitiator){ std::cout << "Exiting WaitForResult state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl; }
void WaitForResult::entry(ContractNetInitiator* contractNetInitiator)
{
  std::cout << "Entering WaitForResult state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;
  std::cout << "WaitForResult state: Waiting for result " << std::endl;
}

void WaitForResult::react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message)
{
  if(Message.getPerformative()=="INFORM")
  {
    contractNetInitiator->informs.push_back(Message);
    contractNetInitiator->handleInform(Message);
  }
  else if(Message.getPerformative()=="FAILURE")
  {
    contractNetInitiator->informs.push_back(Message);
    contractNetInitiator->handleFailure(Message);
  }
  else
  {
    contractNetInitiator->informs.push_back(Message);
    std::cout << "WaitForResult state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message);
    //send a NOT-UNDERSTOOD msg to communicate communication problems.
  }

  std::cout << "WaitForResult state: No. of informs is " << contractNetInitiator->informs.size() << " No. of acceptances is " << contractNetInitiator->nAcceptances << std::endl;
  if(contractNetInitiator->informs.size() == contractNetInitiator->nAcceptances)
  {
    contractNetInitiator->handleAllResultNotifications(contractNetInitiator->informs);
    contractNetInitiator->setState(EndProtocol::getInstance());
  }
}
// ----------------------------------------------------------------------------
// State: EvaluateBids
//
void EvaluateBids::react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message){}
void EvaluateBids::exit(ContractNetInitiator* contractNetInitiator){ std::cout << "Exiting EvaluateBids state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl; }
void EvaluateBids::entry(ContractNetInitiator* contractNetInitiator)
{
  std::cout << "Entering EvaluateBids state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;

  contractNetInitiator->acceptances = contractNetInitiator->handleAllResponses(contractNetInitiator->responses);
  //send(contractNetInitiator->acceptances)
  for(unsigned int i = 0; i < contractNetInitiator->acceptances.size(); i++)
  {
    if(contractNetInitiator->acceptances[i].getPerformative()=="ACCEPT")
      {
        contractNetInitiator->nAcceptances++;
      }
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
void StoreBids::entry(ContractNetInitiator* contractNetInitiator){std::cout << "Entering StoreBids state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;}
void StoreBids::exit(ContractNetInitiator* contractNetInitiator){std::cout << "Exiting StoreBids state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;}

void StoreBids::react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message)
{
  if(Message.getPerformative()=="PROPOSE")
  {
    contractNetInitiator->responses.push_back(Message);
    contractNetInitiator->handlePropose(Message);
  }
  else if(Message.getPerformative()=="REJECT")
  {
    contractNetInitiator->responses.push_back(Message);
    contractNetInitiator->handleReject(Message);
  }
  else
  {
    contractNetInitiator->responses.push_back(Message);
    std::cout << "StoreBids state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message);
    //send a NOT-UNDERSTOOD msg to communicate communication problems.
  }

  if(contractNetInitiator->responses.size() == contractNetInitiator->nResponders)
    contractNetInitiator->setState(EvaluateBids::getInstance());
}

// ----------------------------------------------------------------------------
// State: SendCfp
//
void SendCfp::entry(ContractNetInitiator* contractNetInitiator){std::cout << "Entering SendCfp state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;}
void SendCfp::exit(ContractNetInitiator* contractNetInitiator){std::cout << "Exiting SendCfp state ... " << "nResponders: " << contractNetInitiator->nResponders << " nAcceptances: " << contractNetInitiator->nAcceptances << std::endl;}
void SendCfp::react(ContractNetInitiator* contractNetInitiator, ACLMessage const & Message)
{
  if (Message.getPerformative() == "CFP") //TO-DO: && Message.getSender() == "this agent identifier" change it with this agent's ID parameter
    {
      contractNetInitiator->cfp = Message;
      contractNetInitiator->nResponders = Message.getReceivers().size();
      if(contractNetInitiator->nResponders > 0)
      {
        std::cout << "SendCfp state: CFP message to be sent to " << contractNetInitiator->nResponders << " agents" << std::endl;
        //send(CFP)
        contractNetInitiator->setState(StoreBids::getInstance());
      }
      else
        std::cout << "SendCfp state: CFP message has " << contractNetInitiator->nResponders << " receiver agents." << std::endl;
    }
  else
    {
      std::cout << "SendCfp state: Message passed to this behaviour should be CFP and AID == this agent's AID" << std::endl;
      //send a NOT-UNDERSTOOD msg to communicate communication problems.
    }
}