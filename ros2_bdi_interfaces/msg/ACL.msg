# FIPA Communicative Acts / Performative
string ACCEPT_PROPOSAL = "accept-proposal"
string AGREE = "agree"
string CANCEL = "cancel"
string CALL_FOR_PROPOSAL = "cfp"
string CONFIRM = "confirm"
string DISCONFIRM = "disconfirm"
string FAILURE = "failure"
string INFORM = "inform"
string INFORM_IF = "inform-if"
string INFORM_REF = "inform-ref"
string NOT_UNDERSTOOD = "not-understood"
string PROPAGATE = "propagate"
string PROPOSE = "propose"
string PROXY = "proxy"
string QUERY_IF = "query-if"
string QUERY_REF = "query-ref"
string REFUSE = "refuse"
string REJECT_PROPOSAL = "reject-proposal"
string REQUEST = "request"
string REQUEST_WHEN = "request-when"
string REQUEST_WHENEVER = "request-whenever"
string SUBSCRIBE = "subscribe"

# FIPA Protocols
string FIPA_REQUEST = "fipa-request"
string FIPA_QUERY = "fipa-query"
string FIPA_REQUEST_WHEN = "fipa-request-when"
string FIPA_CONTRACT_NET = "fipa-contract-net"
string FIPA_ITERATED_CONTRACT_NET = "fipa-iterated-contract-net"
string FIPA_BROKERING = "fipa-brokering"
string FIPA_RECRUITING = "fipa-recruiting"
string FIPA_SUBSCRIBE = "fipa-subscribe"
string FIPA_PROPOSE = "fipa-propose"

# FIPA ACL Message Parameters
string performative
string sender
string[] receiver
string reply_to
string content
string language
string encoding
string ontology
string protocol
string conversation_id
string reply_with
string in_reply_to
float32 reply_by
