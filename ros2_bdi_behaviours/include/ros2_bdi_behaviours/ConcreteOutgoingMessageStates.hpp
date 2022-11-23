#pragma once
#include "ros2_bdi_behaviours/OutgoingMessageState.hpp"
#include "ros2_bdi_behaviours/OutgoingMessage.hpp"

namespace OutgoingStates
{
	// ----------------------------------------------------------------------------
	// State: EndProtocol
	//
	class EndProtocol : public OutgoingMessageState
	{
	public:
		void entry(OutgoingMessage* outgoingMessage);
		void react(OutgoingMessage* outgoingMessage, ACLMessage const & Message);
		void exit(OutgoingMessage* outgoingMessage);
		static OutgoingMessageState& getInstance(){ static EndProtocol singleton; return singleton; }

	private:
		EndProtocol() {}									//Hide the constructor
		EndProtocol(const EndProtocol& other);				//Hide the copy constructor
		EndProtocol& operator=(const EndProtocol& other);	//Hide the assignment operator
	};

	// ----------------------------------------------------------------------------
	// State: SendMessage
	//
	class SendMessage : public OutgoingMessageState
	{
	public:
		void entry(OutgoingMessage* outgoingMessage);
		void react(OutgoingMessage* outgoingMessage, ACLMessage const & Message);
		void exit(OutgoingMessage* outgoingMessage);
		static OutgoingMessageState& getInstance(){ static SendMessage singleton; return singleton; }

	private:
		SendMessage() {}									//Hide the constructor
		SendMessage(const SendMessage& other);			//Hide the copy constructor
		SendMessage& operator=(const SendMessage& other);	//Hide the assignment operator
	};
}