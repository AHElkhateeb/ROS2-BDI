#include "ros2_bdi_behaviours/TransporterBehaviour.hpp"

#define TETRABOT_SPEED 1500		//Speed in mm/s

/*
Helper functions
*/
static std::vector<std::string> split(const std::string& s, char seperator= ' '){	std::vector<std::string> output;	std::string::size_type prev_pos = 0, pos = 0;	while((pos = s.find(seperator, pos)) != std::string::npos){	std::string substring( s.substr(prev_pos, pos-prev_pos) );	output.push_back(substring);	prev_pos = ++pos;	}	output.push_back(s.substr(prev_pos, pos-prev_pos));	return output;	}

static std::vector<float> get_coordinates(const std::string& waypoint)
    {
        if(waypoint == "wp_equip")
            return {500, 500};
        else if(waypoint == "wp_pipe")
            return {500, 9500};
        else if(waypoint == "wp_charge")
            return {6500, 500};
        else if(waypoint == "wp_toolchange")
            return {3500, 500};
        else if(waypoint == "wp_seat")
            return {9500, 500};
        else if(waypoint == "wp_fuselage")
            return {9500, 9500};
        else
            return {(float)((waypoint[3] - '0')*1000 + 500), (float)((waypoint[4] - '0')*1000 + 500)};
    }

static float get_x(const std::string& waypoint)
    {
		return get_coordinates(waypoint)[0];
    }

static float get_y(const std::string& waypoint)
    {
		return get_coordinates(waypoint)[1];
    }

static int DurationCostBetween(const std::string& wp1,const std::string& wp2)
	{
		return sqrt( pow((double)(get_x(wp1)-get_x(wp2)), 2) + pow((double)(get_y(wp1)-get_y(wp2)), 2) ) / TETRABOT_SPEED;
	}

/*
Constructor
*/

TransporterBehaviour::TransporterBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id) : ContractNetResponder(desire_set, belief_set, agent_id)
{}

/*
Specific to implementation
*/

ACLMessage TransporterBehaviour::handleCfp(ACLMessage cfp)
{
	// Call for proposal content = transport payload wp_from wp_to tool_required robots_required_number_in_text

	RCLCPP_INFO(node_->get_logger(), "Received a call for proposal from agent "+cfp.getSender()+" for "+cfp.getContent());

	ACLMessage propose = cfp.createReply();
	
	int duration_cost = 100;
	bool RequiredToolMounted;
	bool EnoughBattery;

	// Call for proposal content = transport payload wp_from wp_to tool_required robots_required_number_in_text
	std::vector<std::string> task = split(cfp.getContent(), ' ');

	std::string wp_payload = task[2];
	std::string wp_to = task[3];
	
	//ManagedBelief(const std::string& name,const int& pddl_type,const ManagedType& type);
	//name:"tool_mounted" params:"tetrabotID" "tool"

	std::string ToolMounted = getBelief("tool_mounted",{agent_id_}).params[1];
	int Battery = getBelief("battery_charge",{agent_id_}).value;
	std::string current_wp = getBelief("in",{agent_id_}).params[1];

	RequiredToolMounted = (ToolMounted == task[4]);
	EnoughBattery = (Battery >= 30);
	
	if(RequiredToolMounted && EnoughBattery)
		duration_cost= DurationCostBetween(current_wp, wp_payload);
	else if(!RequiredToolMounted && EnoughBattery)
		duration_cost= DurationCostBetween(current_wp, "wp_toolchange") + 2 + DurationCostBetween("wp_toolchange", wp_payload);
	else if(!EnoughBattery && RequiredToolMounted)
		duration_cost= DurationCostBetween(current_wp, "wp_charge") + 4 + DurationCostBetween("wp_charge", wp_payload);
	else 
		duration_cost= DurationCostBetween(current_wp, "wp_charge") + 4 + DurationCostBetween("wp_charge", "wp_toolchange") + 2 + DurationCostBetween("wp_toolchange", wp_payload);

	propose.setContent( std::to_string(duration_cost) );


	if(duration_cost > 20) // greater than 20 seconds
	{
		propose.setPerformative(AclMsg::REFUSE);
		RCLCPP_INFO(node_->get_logger(), "Refused to send a bid because the cost is "+std::to_string(duration_cost)+" which is higher than 20");
	}
	else
	{
		propose.setPerformative(AclMsg::PROPOSE);
		RCLCPP_INFO(node_->get_logger(), "Sent a bid because the cost is "+std::to_string(duration_cost)+" which is lower than 20");
	}

	return propose;
}

ACLMessage TransporterBehaviour::handleAcceptProposal(ACLMessage cfp, ACLMessage propose, ACLMessage accept)
{
	ACLMessage inform = accept.createReply();
	
	RCLCPP_INFO(node_->get_logger(), "Agent "+accept.getSender()+" accepted the proposal with duration cost of "+propose.getContent()+"seconds");
	
	RCLCPP_INFO(node_->get_logger(), "Agent "+agent_id_+" is transporting the payload ");

	RCLCPP_INFO(node_->get_logger(), "Agent "+agent_id_+"Successfully transported the payload");
	inform.setPerformative(AclMsg::INFORM);
	inform.setContent("Transportation Successful");

	return inform;
}