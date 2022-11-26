#include "ros2_bdi_behaviours/TransporterBehaviour.hpp"

/*Helper function*/
static std::vector<std::string> split(const std::string& s, char seperator= ' '){	std::vector<std::string> output;	std::string::size_type prev_pos = 0, pos = 0;	while((pos = s.find(seperator, pos)) != std::string::npos){	std::string substring( s.substr(prev_pos, pos-prev_pos) );	output.push_back(substring);	prev_pos = ++pos;	}	output.push_back(s.substr(prev_pos, pos-prev_pos));	return output;	}

TransporterBehaviour::TransporterBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id) : ContractNetResponder(desire_set, belief_set, agent_id)
{}

/*
Specific to implementation
*/

ACLMessage TransporterBehaviour::handleCfp(ACLMessage cfp)
{
	ACLMessage propose = cfp.createReply();
	int cost = 100;

	RCLCPP_INFO(node_->get_logger(), "Received a call for proposal from agent "+cfp.getSender()+" for "+cfp.getContent());
	
	propose.setPerformative(AclMsg::PROPOSE);
	propose.setContent( std::to_string(cost) );

	return propose;
}

ACLMessage TransporterBehaviour::handleAcceptProposal(ACLMessage cfp, ACLMessage propose, ACLMessage accept)
{
	ACLMessage inform = accept.createReply();
	
	RCLCPP_INFO(node_->get_logger(), "Agent "+accept.getSender()+" accepted the proposal with cost "+propose.getContent());

	inform.setPerformative(AclMsg::INFORM);
	inform.setContent("Transportation Successful");

	return inform;
}