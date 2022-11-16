#ifndef CONV_CLIENT_PARAMS_H_
#define CONV_CLIENT_PARAMS_H_

#define ACL_SRV "acl_srv"
#define ACL_MSG_TOPIC "acl_msg"
#define DEL_CONV_TOPIC "del_conv"

#define CURRENT_TIME_MILLIS std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())

#endif // CONV_CLIENT_PARAMS_H_
