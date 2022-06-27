#include"ros2_bdi_skills/ACLMessage.hpp"

	ACLMessage::ACLMessage()
	{
		performative_ = "NOT_UNDERSTOOD"; //TO-DO define constants
	}

	ACLMessage::ACLMessage(string performative)
	{
		performative_ = performative;
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

	string ACLMessage::currentTimeMillis() //TO-DO should be moved somewhere else
	{
		auto time = std::chrono::system_clock::now();
		auto since_epoch = time.time_since_epoch();
		auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch);
		return std::to_string(millis.count());
	}

	ACLMessage 	ACLMessage::createReply()
	{
		ACLMessage m = ACLMessage(performative_);
		auto it = getAllReplyTo();
		if(it.first == it.second)
		{ 
			m.addReceiver(sender_); 
		}
		else
		{
			while(it.first != it.second)
			{
				m.addReceiver(*it.first);
				advance(it.first, 1);
			}
		}
		m.setLanguage(language_);
		m.setOntology(ontology_);
		m.setProtocol(protocol_);
		m.setInReplyTo(reply_with_);
		m.setConversationId(conversation_id_);
		
		if (sender_ != "")
			m.setReplyWith(sender_ + currentTimeMillis()); 
		else
			m.setReplyWith("X" + currentTimeMillis());

		return m; 		
	}

	std::pair<vector<string>::iterator, vector<string>::iterator> ACLMessage::getAllReplyTo()
	{
		return std::make_pair(reply_to_.begin(), reply_to_.end());
	}

	string ACLMessage::getContent()
	{
		return content_;
	}

	string ACLMessage::getConversationId()
	{
		return conversation_id_;
	}

	string ACLMessage::getEncoding()
	{
		return encoding_;
	}

	string ACLMessage::getInReplyTo()
	{
		return in_reply_to_;
	}

	string ACLMessage::getLanguage()
	{
		return language_;
	}

	string ACLMessage::getOntology()
	{
		return ontology_;
	}

	string ACLMessage::getPerformative()
	{
		return performative_;
	}

	string ACLMessage::getProtocol()
	{
		return protocol_;
	}

	float ACLMessage::getReplyBy()
	{
		return reply_by_;
	}

	string ACLMessage::getReplyWith()
	{
		return reply_with_;
	}

	string ACLMessage::getSender()
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
		performative_= "NOT_UNDERSTOOD"; //TO-DO define constant;
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

	ACLMessage ACLMessage::shallowClone()
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