#include "ros2_bdi_behaviours/conversations_client.hpp"

#include"ros2_bdi_behaviours/ACLMessage.hpp"

//seconds to wait before giving up on performing any request (service does not appear to be up)
#define WAIT_SRV_UP 1   

//seconds to wait before giving up on waiting for the response
#define WAIT_RESPONSE_TIMEOUT 1

//to use sleep()
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using std::string;

using ACLConversations::ConversationsClient;

//APIs for current plan execution, adding/del belief, adding/del desire, maybe check desire/belief
//adding and delition APIS should return a bool for succesful operation.
//sendMessage() and other one in actions with different behaviour
//Check that the message to be sent as an output of the handler functions follows the protocol message performatives

ConversationsClient::ConversationsClient(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set) : desire_set_{*desire_set} , belief_set_{*belief_set}
{
    // node to perform async request to communication services of queried agent(s)
    // make the node spin just while waiting response or until timeout is reached
    node_ = rclcpp::Node::make_shared("conversations_client");
    RCLCPP_INFO(node_->get_logger(), "ConvID not found, Node Created");
}

void ConversationsClient::receiveMsg(ACLMessage msg)
{
    RCLCPP_INFO(node_->get_logger(), "Before 10 second sleep, ConvID: " + msg.getConversationId());
    int x= (*belief_set_.begin()).pddlType();
    RCLCPP_INFO(node_->get_logger(), "   pddlType: " + std::to_string(x));
    sleep(10);
    RCLCPP_INFO(node_->get_logger(), "After 10 second sleep, ConvID: " + msg.getConversationId());
    x= (*(belief_set_.begin())).pddlType();
    RCLCPP_INFO(node_->get_logger(), "   pddlType: " + std::to_string(x));
    
    ACLMessage reply = msg.createReply();
    reply.setContent("Agree");

}
