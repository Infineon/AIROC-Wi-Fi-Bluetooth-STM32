\startuml{uml_wps_enrollee.png}
title WPS PIN mode sequence

activate APP
APP -> "WiFi Connection Manager": cy_wcm_init
note over APP,"WiFi Connection Manager": Initializes WCM Interface
"WiFi Connection Manager"-->APP: returns cy_rslt_t

APP->"WiFi Connection Manager" : \ncy_wcm_wps_generate_pin
note over APP,"WiFi Connection Manager": Generates random pin
"WiFi Connection Manager"-->APP: returns cy_rslt_t

APP->"WiFi Connection Manager": \ncy_wcm_wps_enrollee
activate "WiFi Connection Manager"
"WiFi Connection Manager"->WPS: cy_wps_init
note over "WiFi Connection Manager",WPS: Initializes WPS workspace
activate WPS
WPS->"WiFi Host Driver": Registers callabck to receive EAPOL packets
"WiFi Host Driver"-->WPS: returns whd_result_t
WPS-->"WiFi Connection Manager": returns cy_rslt_t
deactivate WPS

"WiFi Connection Manager"->WPS:cy_wps_start
note right of WPS: Create WPS thread
WPS-->"WPS Thread": WPS thread created
note right of "WPS Thread": WPS Thread started running
WPS-->"WiFi Connection Manager": return cy_rslt_t

"WiFi Connection Manager"->WPS:cy_wps_wait_till_complete
activate WPS
WPS-->"WPS Thread": Blocks till WPS thread finishes operation
activate "WPS Thread"
note right of "WPS Thread": Start timer for 2 minutes \nStart scanning nearby APs \nTry to join to APs

"WPS Thread"->"WiFi Host Driver":Send EAPOL start
"WiFi Host Driver"->"WPS Thread":EAPOL packet received

note right of "WPS Thread": EAP Request/Identity

"WPS Thread"->"WiFi Host Driver":Send EAP Reponse/Identity

"WiFi Host Driver"->"WPS Thread":EAPOL packet received

note right of "WPS Thread": EAP Request Start

== WPS M1-M8 exchange started ==
"WPS Thread"->"WiFi Host Driver":EAP Response M1 message

"WiFi Host Driver"->"WPS Thread":EAPOL packet received
note right of "WPS Thread": Parse M2 message

"WPS Thread"->"WiFi Host Driver":EAP Response M3 message

"WiFi Host Driver"->"WPS Thread":EAPOL packet received
note right of "WPS Thread": Parse M4 message

"WPS Thread"->"WiFi Host Driver":EAP Response M5 message

"WiFi Host Driver"->"WPS Thread":EAPOL packet received
note right of "WPS Thread": Parse M6 message

"WPS Thread"->"WiFi Host Driver":EAP Response M7 message

"WiFi Host Driver"->"WPS Thread":EAPOL packet received
note right of "WPS Thread": Parse M8 message

"WPS Thread"->"WiFi Host Driver":EAP Response(Done)

== WPS M1-M8 exchange completed ==

note right of "WPS Thread":Set the result to SUCESS, Failure or Timeout \nExit from thread

"WPS Thread"-->WPS:WPS thread terminated
WPS-->"WiFi Connection Manager":WPS thread terminated

"WiFi Connection Manager"->WPS:cy_wps_get_result
WPS-->"WiFi Connection Manager":returns internal wps result

"WiFi Connection Manager"-->APP:convert internal WPS result to WCM result and return

APP->APP: Application receives the credentials.\nAnd connects to AP using credentials received

deactivate APP
\enduml