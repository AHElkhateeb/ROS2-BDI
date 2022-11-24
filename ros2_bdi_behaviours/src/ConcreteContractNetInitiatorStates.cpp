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
  contractNetInitiator->deleteConvID();
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
  if(Message.getPerformative()=="inform")
  {
    contractNetInitiator->informs.push_back(Message);
    contractNetInitiator->handleInform(Message);
  }
  else if(Message.getPerformative()=="failure")
  {
    contractNetInitiator->informs.push_back(Message);
    contractNetInitiator->handleFailure(Message);
  }
  else
  {
    contractNetInitiator->informs.push_back(Message);
    std::cout << "WaitForResult state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message);
    contractNetInitiator->sendMsg(Message.createReply()); //Sends back a NOTUNDERSTOOD Message
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
  for(unsigned int i = 0; i < contractNetInitiator->acceptances.size(); i++)
  {
    if(contractNetInitiator->acceptances[i].getPerformative()=="accept-proposal")
      {
        contractNetInitiator->nAcceptances += contractNetInitiator->acceptances[i].getReceivers().size(); //In one ACL Message there can be multiple receivers
      }
  }

  contractNetInitiator->sendMsg(contractNetInitiator->acceptances);
  
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
  if(Message.getPerformative()=="propose")
  {
    contractNetInitiator->responses.push_back(Message);
    contractNetInitiator->handlePropose(Message);
  }
  else if(Message.getPerformative()=="reject-proposal")
  {
    contractNetInitiator->responses.push_back(Message);
    contractNetInitiator->handleReject(Message);
  }
  else
  {
    contractNetInitiator->responses.push_back(Message);
    std::cout << "StoreBids state: Out of sequence message received" << std::endl;
    contractNetInitiator->handleNotUnderstood(Message);
    contractNetInitiator->sendMsg(Message.createReply()); //Sends back a NOTUNDERSTOOD Message
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
  if (Message.getPerformative() == "cfp") //TO-DO: && Message.getSender() == "this agent identifier" change it with this agent's ID parameter
    {
      contractNetInitiator->cfp = Message;
      contractNetInitiator->nResponders = Message.getReceivers().size();
      if(contractNetInitiator->nResponders > 0)
      {
        std::cout << "SendCfp state: CFP message to be sent to " << contractNetInitiator->nResponders << " agents" << std::endl;
        contractNetInitiator->nResponders = contractNetInitiator->sendMsg(contractNetInitiator->cfp);
        contractNetInitiator->setState(StoreBids::getInstance());
      }
      else
        std::cout << "SendCfp state: CFP message has " << contractNetInitiator->nResponders << " receiver agents." << std::endl;
    }
  else
    {
      std::cout << "SendCfp state: Message passed to this behaviour should be CFP and AID == this agent's AID" << std::endl;
    }
}