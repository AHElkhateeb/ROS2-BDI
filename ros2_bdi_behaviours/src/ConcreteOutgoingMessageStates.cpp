#include "ros2_bdi_behaviours/ConcreteOutgoingMessageStates.hpp"

using namespace OutgoingStates;

// ----------------------------------------------------------------------------
// State: EndProtocol
//
void EndProtocol::exit(OutgoingMessage* outgoingMessage){ std::cout << "Exiting EndProtocol state ... " << std::endl; }
void EndProtocol::react(OutgoingMessage* outgoingMessage, ACLMessage const & Message){}
void EndProtocol::entry(OutgoingMessage* outgoingMessage)
{
  std::cout << "Entering EndProtocol state ... " << std::endl;

  //deletes ConvID SharedPointer
  outgoingMessage->deleteConvID();
}

// ----------------------------------------------------------------------------
// State: SendMessage
//
void SendMessage::entry(OutgoingMessage* outgoingMessage){std::cout << "Entering SendMessage state ... " << std::endl;}
void SendMessage::exit(OutgoingMessage* outgoingMessage){std::cout << "Exiting SendMessage state ... " << std::endl;}
void SendMessage::react(OutgoingMessage* outgoingMessage, ACLMessage const & Message)
{
  std::cout << "SendMessage state: Outgoing message to be sent" << std::endl;
  outgoingMessage->sendMsg(Message);
  outgoingMessage->setState(EndProtocol::getInstance());
}