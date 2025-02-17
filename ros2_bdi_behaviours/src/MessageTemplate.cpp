#include"ros2_bdi_behaviours/MessageTemplate.hpp"

	MessageTemplate::MessageTemplate()
	{
		performative_ = AclMsg::NOT_UNDERSTOOD;
	}

	MessageTemplate::MessageTemplate(ACLMessage msg)
	{
		performative_ = msg.getPerformative();
		sender_ = msg.getSender();
		receiver_ = msg.getReceivers();
		reply_to_ = msg.getReplyTo();
		content_ = msg.getContent();
		language_ = msg.getLanguage();
		encoding_ = msg.getEncoding();
		ontology_ = msg.getOntology();
		protocol_ = msg.getProtocol();
		conversation_id_ = msg.getConversationId();
		reply_with_ = msg.getReplyWith();
		in_reply_to_ = msg.getInReplyTo();
		reply_by_ = msg.getReplyBy();
	}

	void MessageTemplate::reset()
	{
		performative_= AclMsg::NOT_UNDERSTOOD;
		sender_.clear();
		receiver_.clear();
		reply_to_.clear();
		content_.clear(); 
		language_.clear();
		encoding_.clear();
		ontology_.clear();
		protocol_.clear();
		conversation_id_.clear();
		reply_with_.clear();
		in_reply_to_.clear();
		reply_by_= 0.0f;
	}

	void MessageTemplate::matchContent(string content)
	{
		content_= content;	
	}

	void MessageTemplate::matchConversationId(string conversation_id)
	{
		conversation_id_= conversation_id;
	}

	void MessageTemplate::matchEncoding(string encoding)
	{
		encoding_=encoding;
	}

	void MessageTemplate::matchInReplyTo(string in_reply_to)
	{
		in_reply_to_= in_reply_to;
	}

	void MessageTemplate::matchLanguage(string language)
	{
		language_= language;
	}

	void MessageTemplate::matchOntology(string ontology)
	{
		ontology_= ontology;
	}

	void MessageTemplate::matchPerformative(string performative)
	{
		performative_= performative;
	}

	void MessageTemplate::matchProtocol(string protocol)
	{
		protocol_= protocol;
	}

	void MessageTemplate::matchReplyBy(float reply_by)
	{
		reply_by_= reply_by;
	}

	void MessageTemplate::matchReplyWith(string reply_with)
	{
		reply_with_= reply_with;
	}

	void MessageTemplate::matchSender(string sender)
	{
		sender_= sender;
	}

	bool MessageTemplate::isMatch(ACLMessage msg)
	{
		if(sender_ != "")
			if(msg.getSender() != sender_)
				return false;

		if(reply_with_ != "")
			if(msg.getReplyWith() != reply_with_)
				return false;

		if(reply_by_ != 0.0f)
			if(msg.getReplyBy() != reply_by_)
				return false;

		if(protocol_ != "")
			if(msg.getProtocol() != protocol_)
				return false;

		if(performative_ != AclMsg::NOT_UNDERSTOOD)
			if(msg.getPerformative() != performative_)
				return false;

		if(ontology_ != "")
			if(msg.getOntology() != ontology_)
				return false;

		if(language_ != "")
			if(msg.getLanguage() != language_)
				return false;

		if(in_reply_to_ != "")
			if(msg.getInReplyTo() != in_reply_to_)
				return false;

		if(encoding_ != "")
			if(msg.getEncoding() != encoding_)
				return false;

		if(conversation_id_ != "")
			if(msg.getConversationId() != conversation_id_)
				return false;

		if(content_ != "")
			if(msg.getContent() != content_)
				return false;
						
		return true;
	}