#include "ros2_bdi_behaviours/OrganizerBehaviour.hpp"

/*Helper function*/
static std::vector<std::string> split(const std::string& s, char seperator= ' '){	std::vector<std::string> output;	std::string::size_type prev_pos = 0, pos = 0;	while((pos = s.find(seperator, pos)) != std::string::npos){	std::string substring( s.substr(prev_pos, pos-prev_pos) );	output.push_back(substring);	prev_pos = ++pos;	}	output.push_back(s.substr(prev_pos, pos-prev_pos));	return output;	}

OrganizerBehaviour::OrganizerBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id) : ContractNetInitiator(desire_set, belief_set, agent_id)
{}

/*
Specific to implementation
*/
void OrganizerBehaviour::handlePropose(ACLMessage propose)
{
	RCLCPP_INFO(node_->get_logger(), "Agent "+propose.getSender()+" proposed "+propose.getContent());
}

void OrganizerBehaviour::handleReject(ACLMessage reject)
{
	RCLCPP_INFO(node_->get_logger(), "Agent "+reject.getSender()+" refused");
}

void OrganizerBehaviour::handleInform(ACLMessage inform)
{
	RCLCPP_INFO(node_->get_logger(), "received a message from " + inform.getSender() + " stating that: " + cfp.getContent() + ", has been successfully completed");
}

void OrganizerBehaviour::handleAllResultNotifications(std::vector<ACLMessage> resultNotifications)
{
	RCLCPP_INFO(node_->get_logger(), "received all results from transporting agents");
}

std::vector<ACLMessage> OrganizerBehaviour::handleAllResponses(std::vector<ACLMessage> responses)
{
	unsigned int numberOfResponses = responses.size();
	std::vector<ACLMessage> acceptances;
	ACLMessage temp;

	for(unsigned int i=0 ; i < numberOfResponses ; i++)
	{
		if(responses[i].getPerformative() != AclMsg::PROPOSE)
		{
			responses.erase(responses.begin()+i);
		}
	}

	unsigned int numberOfAcceptances = responses.size();

	std::vector<std::string> task = split(cfp.getContent(), ' ');

	int robots_num;
	if (task[5] == "one") robots_num = 1;
    else if (task[5] == "two") robots_num = 2;
    else if (task[5] == "three") robots_num = 3;

	if(numberOfAcceptances < robots_num) 
	{
		RCLCPP_INFO(node_->get_logger(), "Not enough transporting agents are available");
		return acceptances;
	}

	// Evaluate proposals, sorting costs in ascending order
	for(unsigned int i=0 ; i < numberOfAcceptances ; i++) 
	{
		for(unsigned int j=0 ; j+1 < numberOfAcceptances-i ; j++) 
		{
			// Swap elements if first one is greater than second one
			if( stoi(responses[j].getContent()) > stoi(responses[j+1].getContent()) )
			{
				temp = responses[j];
                responses[j] = responses[j + 1];
                responses[j + 1] = temp;
			}
		}
	}

	// Accept the proposal of the best proposers
	for(unsigned int i=0 ; i < robots_num ; i++)
	{
		temp = responses[i].createReply();
		temp.setPerformative(AclMsg::ACCEPT_PROPOSAL);
		acceptances.push_back(temp);
	}
	// Reject the rest
	for(unsigned int i=robots_num ; i < numberOfAcceptances ; i++)
	{
		temp = responses[i].createReply();
		temp.setPerformative(AclMsg::REJECT_PROPOSAL);
		acceptances.push_back(temp);
	}

	return acceptances;
}