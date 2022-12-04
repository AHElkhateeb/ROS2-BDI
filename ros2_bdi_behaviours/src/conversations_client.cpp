#include "ros2_bdi_behaviours/conversations_client.hpp"

#include"ros2_bdi_behaviours/ACLMessage.hpp"

// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Communications (Multi-Agent) Request Handler node
#include "ros2_bdi_behaviours/params/conversations_client_params.hpp"

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
using ros2_bdi_interfaces::msg::AclMsg;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::Desire;

using ACLConversations::ConversationsClient;

//APIs for current plan execution, adding/del belief, adding/del desire, maybe check desire/belief
//adding and delition APIS should return a bool for succesful operation.
//sendMessage() and other one in actions with different behaviour
//Check that the message to be sent as an output of the handler functions follows the protocol message performatives

ConversationsClient::ConversationsClient(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id) : desire_set_{*desire_set} , belief_set_{*belief_set} , agent_id_{agent_id}
{
    // node to perform async request to communication services of queried agent(s)
    // make the node spin just while waiting response or until timeout is reached
    node_ = rclcpp::Node::make_shared("conversations_client"+agent_id_+"_"+ConversationId_);
    RCLCPP_INFO(node_->get_logger(), "ConvID not found, Node Created");

    // del conversation client publisher -> to publish on the topic and alter the conversations set
    del_conv_client_publisher_ = node_->create_publisher<std_msgs::msg::String>(DEL_CONV_TOPIC, 10);

    // add belief publisher -> to publish on the topic and alter the belief set
    add_belief_publisher_ = node_->create_publisher<Belief>(ADD_BELIEF_TOPIC, 10);
    // del belief publisher -> to publish on the topic and alter the belief set
    del_belief_publisher_ = node_->create_publisher<Belief>(DEL_BELIEF_TOPIC, 10);

    // add desire publisher -> to publish on the topic and alter the desire set
    add_desire_publisher_ = node_->create_publisher<Desire>(ADD_DESIRE_TOPIC, 10);
    // del desire publisher -> to publish on the topic and alter the desire set
    del_desire_publisher_ = node_->create_publisher<Desire>(DEL_DESIRE_TOPIC, 10);
}

void ConversationsClient::receiveMsg(ACLMessage msg)
{
    RCLCPP_INFO(node_->get_logger(), agent_id_ + " received a message with convID: " + msg.getConversationId() + " and will take no action");
}

int ConversationsClient::sendMsg(ACLMessage msg)
{
    string topicName;
    int sentMsgs=0;

    //TO-DO: msg.setSender("AGENT_ID"); //This Agent's ID should be added here
    msg.setConversationId(this->ConversationId_); //The Conversation ID should be set here
    msg.setSender(this->agent_id_);
    auto ros2Msg = msg.getMessage();

    for(int i = 0 ; i < msg.getReceivers().size() ; i++)
    {
        topicName = "/" + msg.getReceivers()[i] + "/" + ACL_MSG_TOPIC;
        auto msg_publisher = node_->create_publisher<AclMsg>(topicName, 10);
        if( msg_publisher->get_subscription_count() > 0 )
        {
            msg_publisher->publish(ros2Msg);
            sentMsgs++;
        }
    }
    
    // returns the number of receivers the message has been successfully sent to
    return sentMsgs;
}

int ConversationsClient::sendMsg(std::vector<ACLMessage> msgs)
{
    string topicName;
    int sentMsgs=0;

    auto it_begin =msgs.begin();
    auto it_end =msgs.end();
    if(it_begin == it_end)
    {
        return 0; //The ACLMessage vector is empty
    }
    else
    {
        while(it_begin != it_end)
        {
            //(*it_begin).setSender(AGENT_ID); //The Agent's ID should be added here to sender field
            (*it_begin).setConversationId(this->ConversationId_); //The Conversation ID should be set here
            (*it_begin).setSender(this->agent_id_);
            auto ros2Msg = (*it_begin).getMessage();

            for(int i = 0 ; i < (*it_begin).getReceivers().size() ; i++)
            {
                topicName = "/" + (*it_begin).getReceivers()[i] + "/" + ACL_MSG_TOPIC;
                auto msg_publisher = node_->create_publisher<AclMsg>(topicName, 10);
                if( msg_publisher->get_subscription_count() > 0 )
                {
                    msg_publisher->publish(ros2Msg);
                    sentMsgs++;
                }
            }
            advance(it_begin, 1);
        }
    }

    // returns the number of receivers the message has been successfully send to
    return sentMsgs;
}

void ConversationsClient::deleteConvID()
{
    this->del_conv_client_publisher_->publish(std_msgs::msg::String().set__data(this->ConversationId_));
}

void ConversationsClient::addBelief(ros2_bdi_interfaces::msg::Belief belief)
{
    this->add_belief_publisher_->publish(belief);
}

void ConversationsClient::deleteBelief(ros2_bdi_interfaces::msg::Belief belief)
{
    this->del_belief_publisher_->publish(belief);
}

bool ConversationsClient::checkBelief(ros2_bdi_interfaces::msg::Belief belief)
{
    return belief_set_.count(ManagedBelief{belief}) == 1;
}

ros2_bdi_interfaces::msg::Belief ConversationsClient::getBelief(string name, std::vector<string> params)
{
    int matched_params;

    for(BDIManaged::ManagedBelief b : belief_set_)
    {
        matched_params = 0;
        
        if(b.getName() == name)
        {   
            int bounds = (b.getParams().size() > params.size()) ? params.size() : b.getParams().size();

            for(int i=0 ; i < bounds ; i++)
            {
                if(b.getParams()[i].name == params[i])
                {
                    matched_params++;
                }
            }
            
            if(matched_params==params.size())
            {
                return b.toBelief();
            }
        }
    }

    BDIManaged::ManagedBelief empty_belief;
    return empty_belief.toBelief();
}

void ConversationsClient::addDesire(ros2_bdi_interfaces::msg::Desire desire)
{
    this->add_desire_publisher_->publish(desire);
}

void ConversationsClient::deleteDesire(ros2_bdi_interfaces::msg::Desire desire)
{
    this->del_desire_publisher_->publish(desire);
}

bool ConversationsClient::checkDesire(ros2_bdi_interfaces::msg::Desire desire)
{
    return desire_set_.count(ManagedDesire{desire}) == 1;
}