#ifndef ACL_MESSAGE_H_
#define ACL_MESSAGE_H_

#include <string>
#include <iterator>
#include <vector>
#include <chrono>
#include <algorithm>

#include "ros2_bdi_interfaces/msg/acl_msg.hpp"

using std::string;
using std::vector;
using std::iterator;

class ACLMessage
{
private:
	string performative_; // Denotes the type of the communicative act of the ACL message
	string sender_; // the sender's agent_id
	vector<string> receiver_; // Denotes the identity of the intended recipients of the message.
	vector<string> reply_to_; // This parameter indicates that subsequent messages in this conversation thread are to be directed to the agent named in the reply-to parameter, instead of to the agent named in the sender parameter.
	string content_; // Denotes the content of the message.
	string language_;
	string encoding_;
	string ontology_; // Denotes the ontology(s) used to give a meaning to the symbols in the content expression.
	string protocol_; // Denotes the interaction protocol that the sending agent is employing with this ACL message.
	string conversation_id_; // Introduces an expression (a conversation identifier) which is used to identify the ongoing sequence of communicative acts that together form a conversation.
	string reply_with_; // The reply-with parameter is designed to be used to follow a conversation thread in a situation where multiple dialogues occur simultaneously. For example, if agent i sends to agent j a message which contains:	reply-with <expr>	Agent j will respond with a message containing:	in-reply-to <expr>
	string in_reply_to_;
	float reply_by_= 0.0f; // Denotes a time and/or date expression which indicates the latest time by which the sending agent would like to receive a reply.

public:
	ACLMessage();
	ACLMessage(string performative);
	ACLMessage(ros2_bdi_interfaces::msg::AclMsg msg);
	void addReceiver(string receiver);
	void addReplyTo(string reply_to);
	void clearAllReceiver();
	void clearAllReplyTo();
	ACLMessage 	createReply() const;
	string getContent() const;
	string getConversationId() const;
	string getEncoding() const;
	string getInReplyTo() const;
	string getLanguage() const;
	ros2_bdi_interfaces::msg::AclMsg getMessage() const;
	string getOntology() const;
	string getPerformative() const;
	string getProtocol() const;
	vector<string> getReceivers() const;
	vector<string> getReplyTo() const;
	float getReplyBy() const;
	string getReplyWith() const;
	string getSender() const;
	bool removeReceiver(string receiver);
	bool removeReplyTo(string reply_to);
	void reset();
	void setContent(string content);
	void setConversationId(string conversation_id);
	void setEncoding(string encoding);
	void setInReplyTo(string in_reply_to);
	void setLanguage(string language);
	void setOntology(string ontology);
	void setPerformative(string performative);
	void setProtocol(string protocol);
	void setReplyBy(float reply_by);
	void setReplyWith(string reply_with);
	void setSender(string sender);
	ACLMessage shallowClone() const;
};


#endif // ACL_MESSAGE_H_