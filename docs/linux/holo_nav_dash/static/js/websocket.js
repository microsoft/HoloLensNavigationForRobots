//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------


var _socket = null;
var _wsWrapper = null;

//--------------------------------------------------------------------------------------------
function setupWebSocket()
{
    _wsWrapper = new WebSocketWrapper();
    if (_wsWrapper == null) {
        reportStatus("Failed to create webSocket wrapper object.");
        return;
    }

    _wsWrapper.initialize();;
}
//--------------------------------------------------------------------------------------------
function sendMessage(msg)
{
    if (_wsWrapper == null)
        return;

    _wsWrapper.sendMessage(msg);
}
//--------------------------------------------------------------------------------------------

var messages = null;

//--------------------------------------------------------------------------------------------
function reportStatus(msg)
{
    if (messages == null)
        messages = document.getElementById('lblStatus');

    var shouldScroll = messages.scrollTop + messages.clientHeight === messages.scrollHeight;

    var lbl = document.createElement('div');

    lbl.innerHTML = msg;
    //lbl.style.padding = "4px";
    //lbl.style.color = "#000";
    lbl.style.fontSize = "75%";

    messages.appendChild(lbl);

    if (!shouldScroll) {
        // scroll To Bottom
        messages.scrollTop = messages.scrollHeight;
    }
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper = function ()
{
    this.webSocket = null;   
    this.connected = false;
}

WebSocketWrapper.prototype.constructor = WebSocketWrapper;

//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.initialize = function ()
{
    var host = "ws://" + location.hostname + ":" + location.port + "/ws";

    try
    {
        reportStatus('Connecting to "' + host + '"...');

        this.webSocket = new WebSocket(host);

        this.webSocket.onopen = this.onWebSocketOpen.bind(this);
        this.webSocket.onclose = this.onWebSocketClose.bind(this);
        this.webSocket.onmessage = this.onWebSocketMessage.bind(this);
        this.webSocket.onerror = this.onWebSocketError.bind(this);
    }
    catch (exception)
    {
        reportStatus('WebSocket: exception');
        if (window.console)
            console.log(exception);
    }
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.onWebSocketOpen = function (event)
{
    this.connected = true;
    reportStatus('WebSocket: connection opened.');
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.onWebSocketClose = function (event)
{
    var reason = "";

    //
    // only error code 1006 is dispatched: https://www.w3.org/TR/websockets/#concept-websocket-close-fail
    //
    if (event.code == 1000)
        reason = "Normal closure, meaning that the purpose for which the connection was established has been fulfilled.";
    else if (event.code == 1001)
        reason = "An endpoint is \"going away\", such as a server going down or a browser having navigated away from a page.";
    else if (event.code == 1002)
        reason = "An endpoint is terminating the connection due to a protocol error";
    else if (event.code == 1003)
        reason = "An endpoint is terminating the connection because it has received a type of data it cannot accept (e.g., an endpoint that understands only text data MAY send this if it receives a binary message).";
    else if (event.code == 1004)
        reason = "Reserved. The specific meaning might be defined in the future.";
    else if (event.code == 1005)
        reason = "No status code was actually present.";
    else if (event.code == 1006) {
        if (this.connected)
            reason = "The connection was closed abnormally.";
        else
            reason = "The connection could not be established.";
    } else if (event.code == 1007)
        reason = "An endpoint is terminating the connection because it has received data within a message that was not consistent with the type of the message (e.g., non-UTF-8 [http://tools.ietf.org/html/rfc3629] data within a text message).";
    else if (event.code == 1008)
        reason = "An endpoint is terminating the connection because it has received a message that \"violates its policy\". This reason is given either if there is no other sutible reason, or if there is a need to hide specific details about the policy.";
    else if (event.code == 1009)
        reason = "An endpoint is terminating the connection because it has received a message that is too big for it to process.";
    else if (event.code == 1010) // Note that this status code is not used by the server, because it can fail the WebSocket handshake instead.
        reason = "An endpoint (client) is terminating the connection because it has expected the server to negotiate one or more extension, but the server didn't return them in the response message of the WebSocket handshake. <br /> Specifically, the extensions that are needed are: " + event.reason;
    else if (event.code == 1011)
        reason = "A server is terminating the connection because it encountered an unexpected condition that prevented it from fulfilling the request.";
    else if (event.code == 1015)
        reason = "The connection was closed due to a failure to perform a TLS handshake (e.g., the server certificate can't be verified).";
    else
        reason = "Unknown reason";

    reportStatus('WebSocket: ' + reason);

    this.connected = false;
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.onWebSocketMessage = function (event)
{
    // reportStatus('WebSocket: message received: ' + event.data);

    if (typeof event === "undefined" || typeof event.data !== "string")
        return;

    var data = JSON.parse(event.data);

    // console.log("socket msg: '" + data.msgType + "'");
    if (data.msgType == 'initialization') {
    } else if (data.msgType == 'status') {
        setROSNodesStatus(data.status);
    }
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.onWebSocketError = function (event)
{
    //reportStatus('<span style = "color: red;">WebSocket Error: ' + event.data + '</span>');
    //console.log(event);
}
//--------------------------------------------------------------------------------------------
WebSocketWrapper.prototype.sendMessage = function (msg)
{
    if (this.connected)
        this.webSocket.send(msg);
}
