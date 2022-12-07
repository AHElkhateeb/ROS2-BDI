#include"ros2_bdi_behaviours/ACLMessage.hpp"

#define CURRENT_TIME_MILLIS std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

using ros2_bdi_interfaces::msg::AclMsg;

	ACLMessage::ACLMessage()
	{
		performative_ = AclMsg::NOT_UNDERSTOOD;
	}

	ACLMessage::ACLMessage(string performative)
	{
		performative_ = performative;
	}

	ACLMessage::ACLMessage(AclMsg msg)
	{
		performative_ = msg.performative;
		sender_ = msg.sender;
		receiver_ = msg.receiver;
		reply_to_ = msg.reply_to;
		content_ = msg.content;
		language_ = msg.language;
		encoding_ = msg.encoding;
		ontology_ = msg.ontology;
		protocol_ = msg.protocol;
		conversation_id_ = msg.conversation_id;
		reply_with_ = msg.reply_with;
		in_reply_to_ = msg.in_reply_to;
		reply_by_ = msg.reply_by;
	}

	void ACLMessage::addReceiver(string receiver)
	{
		auto itr = std::find(receiver_.begin(), receiver_.end(), receiver);
		if (itr == receiver_.end()) 
		{
			receiver_.push_back(receiver);
		}
	}

	void ACLMessage::addReplyTo(string reply_to)
	{
		auto itr = std::find(reply_to_.begin(), reply_to_.end(), reply_to);
		if (itr == reply_to_.end()) 
		{
			reply_to_.push_back(reply_to);
		}
	}

	void ACLMessage::clearAllReceiver()
	{
		receiver_.clear();
	}

	void ACLMessage::clearAllReplyTo()
	{
		reply_to_.clear();
	}

	ACLMessage 	ACLMessage::createReply() const
	{
		ACLMessage m = ACLMessage();
		auto it_begin =reply_to_.begin();
		auto it_end =reply_to_.end();
		if(it_begin == it_end)
		{ 
			m.addReceiver(sender_); 
		}
		else
		{
			while(it_begin != it_end)
			{
				m.addReceiver(*it_begin);
				advance(it_begin, 1);
			}
		}
		m.setLanguage(language_);
		m.setOntology(ontology_);
		m.setProtocol(protocol_);
		m.setInReplyTo(reply_with_);
		m.setConversationId(conversation_id_);
		
		if (sender_ != "")
			m.setReplyWith(sender_ + CURRENT_TIME_MILLIS); 
		else
			m.setReplyWith("X" + CURRENT_TIME_MILLIS);

		return m; 		
	}

	string ACLMessage::getContent() const
	{
		return content_;
	}

	string ACLMessage::getConversationId() const
	{
		return conversation_id_;
	}

	string ACLMessage::getEncoding() const
	{
		return encoding_;
	}

	string ACLMessage::getInReplyTo() const
	{
		return in_reply_to_;
	}

	string ACLMessage::getLanguage() const
	{
		return language_;
	}

	AclMsg ACLMessage::getMessage() const
	{
		auto msg = AclMsg();
		msg.performative = performative_;
		msg.sender = sender_;
		msg.receiver = receiver_;
		msg.reply_to = reply_to_;
		msg.content = content_;
		msg.language = language_;
		msg.encoding = encoding_;
		msg.ontology = ontology_;
		msg.protocol = protocol_;
		msg.conversation_id = conversation_id_;
		msg.reply_with = reply_with_;
		msg.in_reply_to = in_reply_to_;
		msg.reply_by = reply_by_;
		return msg;
	}

	string ACLMessage::getOntology() const
	{
		return ontology_;
	}

	string ACLMessage::getPerformative() const
	{
		return performative_;
	}

	string ACLMessage::getProtocol() const
	{
		return protocol_;
	}

	vector<string> ACLMessage::getReceivers() const
	{
		return receiver_;
	}
	
	vector<string> ACLMessage::getReplyTo() const
	{
		return reply_to_;
	}

	float ACLMessage::getReplyBy() const
	{
		return reply_by_;
	}

	string ACLMessage::getReplyWith() const
	{
		return reply_with_;
	}

	string ACLMessage::getSender() const
	{
		return sender_;
	}

	bool ACLMessage::removeReceiver(string receiver)
	{
		auto itr = std::find(receiver_.begin(), receiver_.end(), receiver);
		if (itr != receiver_.end()) 
		{
			receiver_.erase(itr);
			return true;
		}
		return false;
	}

	bool ACLMessage::removeReplyTo(string reply_to)
	{
		auto itr = std::find(reply_to_.begin(), reply_to_.end(), reply_to);
		if (itr != reply_to_.end()) 
		{
			reply_to_.erase(itr);
			return true;
		}
		return false;
	}

	void ACLMessage::reset()
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

	void ACLMessage::setContent(string content)
	{
		content_= content;
	}

	void ACLMessage::setConversationId(string conversation_id)
	{
		conversation_id_= conversation_id;
	}

	void ACLMessage::setEncoding(string encoding)
	{
		encoding_=encoding;
	}

	void ACLMessage::setInReplyTo(string in_reply_to)
	{
		in_reply_to_= in_reply_to;
	}

	void ACLMessage::setLanguage(string language)
	{
		language_= language;
	}

	void ACLMessage::setOntology(string ontology)
	{
		ontology_= ontology;
	}

	void ACLMessage::setPerformative(string performative)
	{
		performative_= performative;
	}

	void ACLMessage::setProtocol(string protocol)
	{
		protocol_= protocol;
	}

	void ACLMessage::setReplyBy(float reply_by)
	{
		reply_by_= reply_by;
	}

	void ACLMessage::setReplyWith(string reply_with)
	{
		reply_with_= reply_with;
	}

	void ACLMessage::setSender(string sender)
	{
		sender_= sender;
	}

	ACLMessage ACLMessage::shallowClone() const
	{
		ACLMessage result = ACLMessage(performative_);
		//TO-DO most probably wrong access of private variables use methods instead
		result.sender_ = sender_;
		result.receiver_ = receiver_;
		result.reply_to_ = reply_to_;
		result.content_ = content_;
		result.language_ = language_;
		result.encoding_ = encoding_;
		result.ontology_ = ontology_;
		result.protocol_ = protocol_;
		result.conversation_id_ = conversation_id_;		
		result.reply_with_ = reply_with_;
		result.in_reply_to_ = in_reply_to_;
		result.reply_by_ = reply_by_;
		
		return result;
	}