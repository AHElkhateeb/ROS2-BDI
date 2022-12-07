#ifndef CONVERSATIONS_CLIENT_H_
#define CONVERSATIONS_CLIENT_H_

#include <algorithm>
#include <string>
#include <memory>

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"
#include "ros2_bdi_interfaces/msg/acl_msg.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_bdi_behaviours/ACLMessage.hpp"

namespace ACLConversations{

    /*
        Helping node to avoid blocking situation in ACLCommunicator node when dealing with msgs in conversations
    */
    class ConversationsClient
    {
        public:
            ConversationsClient(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId);

            virtual void receiveMsg(ACLMessage msg);

            int sendMsg(ACLMessage msg);
            int sendMsg(std::vector<ACLMessage> msgs);

            void addBelief(ros2_bdi_interfaces::msg::Belief belief);
            void deleteBelief(ros2_bdi_interfaces::msg::Belief belief);
            bool checkBelief(ros2_bdi_interfaces::msg::Belief belief);
            ros2_bdi_interfaces::msg::Belief getBelief(string name, std::vector<string> params);

            void addDesire(ros2_bdi_interfaces::msg::Desire desire);
            void deleteDesire(ros2_bdi_interfaces::msg::Desire desire);
            bool checkDesire(ros2_bdi_interfaces::msg::Desire desire);

            //del_conv publisher
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr del_conv_client_publisher_;
            //add belief topic publisher
            rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;
            //del belief topic publisher
            rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;
            //add desire topic publisher
            rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr add_desire_publisher_;
            //del desire topic publisher
            rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr del_desire_publisher_;

            void deleteConvID();

        protected:
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