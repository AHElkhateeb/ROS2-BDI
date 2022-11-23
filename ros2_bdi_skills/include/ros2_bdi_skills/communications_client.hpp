#ifndef COMMUNICATIONS_CLIENT_H_
#define COMMUNICATIONS_CLIENT_H_

#include <algorithm>
#include <string>
#include <memory>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/srv/check_belief.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/check_desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"

#include "ros2_bdi_interfaces/msg/acl_msg.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ros2_bdi_skills/communications_structs.hpp"

namespace BDICommunications{

    /*
        Helping node to avoid blocking situation in BDIActionExecutor node when sending a request to a communication
        service of another agent (CHECK/WRITE belief/desire operations)
    */
    class CommunicationsClient
    {
        public:
            CommunicationsClient();

            /*
                Sending CHECK belief request
                    @agent_ref   -> id of the queried agent
                    @agent_group -> agent group of the requesting agent
                    @belief     -> belief to be checked within the belief set of the queried agent
            */
            BDICommunications::CheckBeliefResult checkBeliefRequest(const std::string& agent_ref, 
                    const std::string& agent_group, const ros2_bdi_interfaces::msg::Belief& belief);

            /*
                Sending WRITE belief request
                    @agent_ref   -> id of the queried agent
                    @agent_group -> agent group of the requesting agent
                    @belief     -> belief to be added/modified/deleted within the belief set of the queried agent
                    @op         -> operation to be performed (ADD/DEL), n.b. ADD works as update if belief (FLUENT type) already there
                                    
                ADD successful if belief already there, DELETE successful if belief is already not present
            */
            BDICommunications::UpdBeliefResult updBeliefRequest(const std::string& agent_ref, 
                    const std::string& agent_group, const ros2_bdi_interfaces::msg::Belief& belief, const UpdOperation& op);
            


            /*
                Sending CHECK desire request
                    @agent_ref   -> id of the queried agent
                    @agent_group -> agent group of the requesting agent
                    @desire     -> desire to be checked within the desire set of the queried agent
            */
            BDICommunications::CheckDesireResult checkDesireRequest(const std::string& agent_ref, 
                    const std::string& agent_group, const ros2_bdi_interfaces::msg::Desire& desire);
            /*
                Sending WRITE desire request
                    @agent_ref   -> id of the queried agent
                    @agent_group -> agent group of the requesting agent
                    @desire     -> desire to be added/modified/deleted within the desire set of the queried agent
                    @op         -> operation to be performed (ADD/DEL)
                
                ADD successful if desire already there, DELETE successful if desire is already not present
            */
            BDICommunications::UpdDesireResult updDesireRequest(const std::string& agent_ref, 
                    const std::string& agent_group, const ros2_bdi_interfaces::msg::Desire& desire, const UpdOperation& op);

            /*
                Helper method to publish the ACL message
            */
            bool publishMsg(ros2_bdi_interfaces::msg::AclMsg msg, std::string topic);

        private:
            // node to be spinned while making request
            rclcpp::Node::SharedPtr node_;

            // below client instances to be instantiated while making a request to ...

            // ... @agent_ref/check_belief_srv
            rclcpp::Client<ros2_bdi_interfaces::srv::CheckBelief>::SharedPtr ck_belief_client_;

            // ... @agent_ref/check_desire_srv
            rclcpp::Client<ros2_bdi_interfaces::srv::CheckDesire>::SharedPtr ck_desire_client_;

            // ... @agent_ref/add_belief_srv OR  ... @agent_ref/del_belief_srv
            rclcpp::Client<ros2_bdi_interfaces::srv::UpdBeliefSet>::SharedPtr upd_belief_client_;

            // ... @agent_ref/add_desire_srv OR  ... @agent_ref/del_desire_srv
            rclcpp::Client<ros2_bdi_interfaces::srv::UpdDesireSet>::SharedPtr upd_desire_client_;
    };
};

#endif  // COMMUNICATIONS_CLIENT_H_