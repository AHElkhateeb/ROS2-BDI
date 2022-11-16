#ifndef CONVERSATIONS_CLIENT_H_
#define CONVERSATIONS_CLIENT_H_

#include <algorithm>
#include <string>
#include <memory>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_interfaces/msg/acl_msg.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include"ros2_bdi_behaviours/ACLMessage.hpp"

namespace ACLConversations{

    /*
        Helping node to avoid blocking situation in ACLCommunicator node when dealing with msgs in conversations
    */
    class ConversationsClient
    {
        public:
            ConversationsClient(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id);

            virtual void receiveMsg(ACLMessage msg);

            int sendMsg(ACLMessage msg);
            int sendMsg(std::vector<ACLMessage> msgs);

            //del_conv publisher
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr del_conv_client_publisher_;

            void deleteConvID();

        private:
            // node to be spinned while making request
            rclcpp::Node::SharedPtr node_;
            string ConversationId_;
            string agent_id_;
            const std::set<BDIManaged::ManagedDesire>& desire_set_; 
            const std::set<BDIManaged::ManagedBelief>& belief_set_;

            // below client instances to be instantiated while making a request to ...

    };
};

#endif  // CONVERSATIONS_CLIENT_H_