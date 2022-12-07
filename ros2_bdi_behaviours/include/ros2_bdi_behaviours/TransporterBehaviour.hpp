#pragma once
#include "ros2_bdi_behaviours/ContractNetResponder.hpp"
#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include <math.h>


class TransporterBehaviour : public ContractNetResponder
{
public:
	TransporterBehaviour(std::set<BDIManaged::ManagedDesire>* desire_set, std::set<BDIManaged::ManagedBelief>* belief_set, string &agent_id, string ConversationId);

	//Functions to implement from CNetResponder
	ACLMessage handleCfp(ACLMessage cfp);
	ACLMessage handleAcceptProposal(ACLMessage cfp, ACLMessage propose, ACLMessage accept);
};