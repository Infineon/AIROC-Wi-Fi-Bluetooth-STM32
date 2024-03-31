var group__mqtt__struct =
[
    [ "cy_mqtt_subscribe_info_t", "structcy__mqtt__subscribe__info__t.html", [
      [ "qos", "structcy__mqtt__subscribe__info__t.html#aa4b608145f6dca9d3740d43d83a5f960", null ],
      [ "topic", "structcy__mqtt__subscribe__info__t.html#a7233b6d440d794a302673089552f7acb", null ],
      [ "topic_len", "structcy__mqtt__subscribe__info__t.html#a2bd950023cbd33864c6bc6343af7b06b", null ],
      [ "allocated_qos", "structcy__mqtt__subscribe__info__t.html#aab71d487d9ba575b91bb7845e288465d", null ]
    ] ],
    [ "cy_mqtt_publish_info_t", "structcy__mqtt__publish__info__t.html", [
      [ "qos", "structcy__mqtt__publish__info__t.html#ac3dd00abb3e55019b0f41f169f047d65", null ],
      [ "retain", "structcy__mqtt__publish__info__t.html#acee256975b49014ea4edf38780759084", null ],
      [ "dup", "structcy__mqtt__publish__info__t.html#a8f8b93a8110e01a63fdfee9d89464eb0", null ],
      [ "topic", "structcy__mqtt__publish__info__t.html#aacdf24feaf4d2ceb5c055e50d26ce432", null ],
      [ "topic_len", "structcy__mqtt__publish__info__t.html#ad6c63ce8dd222f55eb6bb3825bee57af", null ],
      [ "payload", "structcy__mqtt__publish__info__t.html#a766647fe8a245f81ff5b2ed5ab57e556", null ],
      [ "payload_len", "structcy__mqtt__publish__info__t.html#a3642a0462fa346cfcc8d114ba78a425e", null ]
    ] ],
    [ "cy_mqtt_broker_info_t", "structcy__mqtt__broker__info__t.html", [
      [ "hostname", "structcy__mqtt__broker__info__t.html#a1573327c42b9dd49610c55aa303e7fea", null ],
      [ "hostname_len", "structcy__mqtt__broker__info__t.html#a38a8850406cc4f8e267e67837a23cd41", null ],
      [ "port", "structcy__mqtt__broker__info__t.html#ad90a2e276913e9598673e31d55ce808e", null ]
    ] ],
    [ "cy_mqtt_connect_info_t", "structcy__mqtt__connect__info__t.html", [
      [ "client_id", "structcy__mqtt__connect__info__t.html#a3df6c6eb014ef6e7566616ece9fa5b1b", null ],
      [ "client_id_len", "structcy__mqtt__connect__info__t.html#ad6ff55a3dfeb52324950c2447d9c8c4e", null ],
      [ "username", "structcy__mqtt__connect__info__t.html#a001bec6fbce4e77575d99f23c12c0d08", null ],
      [ "username_len", "structcy__mqtt__connect__info__t.html#a9b0570bc04fe78767b65245a3ff5fffc", null ],
      [ "password", "structcy__mqtt__connect__info__t.html#a07d6d639ddd2e235ccb0803e8b511623", null ],
      [ "password_len", "structcy__mqtt__connect__info__t.html#ad4023b765e482aebef78aa7d6cdbf8a0", null ],
      [ "clean_session", "structcy__mqtt__connect__info__t.html#a65510d0838eef269db2bd8f2735e1843", null ],
      [ "keep_alive_sec", "structcy__mqtt__connect__info__t.html#a40ded02bb2c432ed7080c1589ebef8e5", null ],
      [ "will_info", "structcy__mqtt__connect__info__t.html#a0238ef1799952c8eeaf4c61c9187c3a6", null ]
    ] ],
    [ "cy_mqtt_message_t", "structcy__mqtt__message__t.html", [
      [ "packet_id", "structcy__mqtt__message__t.html#ae158e70e0ed56cfa2fe80a2e6407b1bc", null ],
      [ "received_message", "structcy__mqtt__message__t.html#aff56023a8b91ffc90fd9e02c84daa894", null ]
    ] ],
    [ "cy_mqtt_event_t", "structcy__mqtt__event__t.html", [
      [ "type", "structcy__mqtt__event__t.html#a9823ea0788cb4e0bd82efd19d195b13e", null ],
      [ "reason", "structcy__mqtt__event__t.html#abbc35b761acab1cda2d4f85418f9c32e", null ],
      [ "pub_msg", "structcy__mqtt__event__t.html#a73dadd9c182b0786d9ed11fb7bb91c65", null ],
      [ "data", "structcy__mqtt__event__t.html#a3f9c7a86ef0794a434c7c1f34605a15f", null ]
    ] ],
    [ "cy_mqtt_received_msg_info_t", "group__mqtt__struct.html#gac201fb43c325826b1b2c83b2eb8052cf", null ],
    [ "cy_mqtt_unsubscribe_info_t", "group__mqtt__struct.html#gafd0d1488be43b96eef830bb7f8c1653e", null ],
    [ "cy_mqtt_qos_t", "group__mqtt__struct.html#ga78bd2ae3cdea69f18ffec6a72bbc28be", [
      [ "CY_MQTT_QOS0", "group__mqtt__struct.html#gga78bd2ae3cdea69f18ffec6a72bbc28beac5d2a5215667dd6c5a2edf4146a50c87", null ],
      [ "CY_MQTT_QOS1", "group__mqtt__struct.html#gga78bd2ae3cdea69f18ffec6a72bbc28beac08dce78d352bab0f02ad032821ccf20", null ],
      [ "CY_MQTT_QOS2", "group__mqtt__struct.html#gga78bd2ae3cdea69f18ffec6a72bbc28bead53bc925a9202da727dcead158c6d890", null ],
      [ "CY_MQTT_QOS_INVALID", "group__mqtt__struct.html#gga78bd2ae3cdea69f18ffec6a72bbc28beaac77e0934620e20e664d3145facf6486", null ]
    ] ],
    [ "cy_mqtt_event_type_t", "group__mqtt__struct.html#ga1cefedb516ade340c9a829ec7b998fba", [
      [ "CY_MQTT_EVENT_TYPE_SUBSCRIPTION_MESSAGE_RECEIVE", "group__mqtt__struct.html#gga1cefedb516ade340c9a829ec7b998fbaac017ae74a362a60128e4bc0482a085de", null ],
      [ "CY_MQTT_EVENT_TYPE_DISCONNECT", "group__mqtt__struct.html#gga1cefedb516ade340c9a829ec7b998fbaa0a7b7d2061d3a6331115e521c16fa0fb", null ]
    ] ],
    [ "cy_mqtt_disconn_type_t", "group__mqtt__struct.html#ga4e499e4898c3baae910860d7ce73f300", [
      [ "CY_MQTT_DISCONN_TYPE_BROKER_DOWN", "group__mqtt__struct.html#gga4e499e4898c3baae910860d7ce73f300a99e819db5a5b4c25e2eda4658e4a2ebb", null ],
      [ "CY_MQTT_DISCONN_TYPE_NETWORK_DOWN", "group__mqtt__struct.html#gga4e499e4898c3baae910860d7ce73f300a434e424ed02e3f4fb2ccad520b7f0805", null ],
      [ "CY_MQTT_DISCONN_TYPE_BAD_RESPONSE", "group__mqtt__struct.html#gga4e499e4898c3baae910860d7ce73f300a67fc4279a46c2520953486ab71cd4884", null ],
      [ "CY_MQTT_DISCONN_TYPE_SND_RCV_FAIL", "group__mqtt__struct.html#gga4e499e4898c3baae910860d7ce73f300a4db84d670967fcbc08d81621427d5130", null ]
    ] ]
];