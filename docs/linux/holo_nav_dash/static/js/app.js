//--------------------------------------------------------------------------------------------
// Copyright(c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// --------------------------------------------------------------------------------------------

var _clrControlEnabled = "#000000";
var _clrControlDisabled = "#a0a0a0";

var _state_unknown = "unknown";
var _state_not_started = "not started";
var _state_running = "running";
var _state_starting = "starting";

var _nodeMatrix = 
[
    { context: "pepper_robot",          lblID: "lblPepperRobot",        btnID: "btnStartPepper",       btnIcon: "toggleBtnPepper" },
    { context: "hololens_ros_bridge",   lblID: "lblHololensROSBridge",  btnID: "btnStartBridge",       btnIcon: "toggleBtnBridge" },
    { context: "anchor_localizer",      lblID: "lblAnchorLocalizer",    btnID: "btnStartAnchor",       btnIcon: "toggleBtnAnchor" },
    { context: "static_calibration",    lblID: "lblStaticCalibration",  btnID: "btnStartCalibration",  btnIcon: "toggleBtnCalibration" },
    { context: "dynamic_adjuster",      lblID: "lblDynamicAdjuster",    btnID: "btnStartAdjuster",     btnIcon: "toggleBtnAdjuster" },
    { context: "localizer",             lblID: "lblLocalizer",          btnID: "btnStartLocalizer",    btnIcon: "toggleBtnLocalizer" }
];

var _lblHololensSequence = null;
var _btnHoloLensCalibration = null;

//--------------------------------------------------------------------------------------------
function onBodyLoad()
{
    //
    // no scrollbars
    document.body.style.overflow = 'hidden';

    setupMiscellaneous();

    //
    // last...
    setupWebSocket();
}
//--------------------------------------------------------------------------------------------
function setupMiscellaneous()
{
    var objBtn;

    for (var i = 0; i < _nodeMatrix.length; i++) {
        //
        // add additional keys
        _nodeMatrix[i]['state'] = _state_unknown;

        objBtn = document.getElementById(_nodeMatrix[i]['btnID']);
        if (objBtn) {
            //
            // disable and hide play/stop buttons.
            if (false) {
                //
                // set up click event
                var idx = i;

                objBtn.addEventListener("click", StartStopNode.bind(this, idx), false);
            } else {
                objBtn.style.display = "none";
            }
        }
    }

    _btnHoloLensCalibration = document.getElementById("btnHoloLensCalibration");
    if (_btnHoloLensCalibration) {
        _btnHoloLensCalibration.disabled = true;
    
        _btnHoloLensCalibration.addEventListener("click", function () {
            var msg = JSON.stringify({ msgType: "calibrateHoloLens" });

            sendMessage(msg);
        });
    }

    objBtn = document.getElementById("btnExitApp");
    if (objBtn) {
        objBtn.addEventListener("click", function () {
            var msg = JSON.stringify({ msgType: "quitApplication" });

            sendMessage(msg);
        });
    }

    _lblHololensSequence = document.getElementById("lblHololensSequence");
}
//--------------------------------------------------------------------------------------------
function EnableDisableUIControl(id, enable, changeColor = false)
{
    var obj = document.getElementById(id);

    if (!obj)
        return;

    if (obj.checked != undefined)
        obj.checked = enable ? true : false;

    if (obj.disabled != undefined)
        obj.disabled = enable ? false : true;

    if (changeColor)
        obj.style.color = enable ? _clrControlEnabled : _clrControlDisabled;
}
//--------------------------------------------------------------------------------------------
function StartStopNode(idx, event)
{
    if ((idx < 0) || (idx >= _nodeMatrix.length)) {
        console.warn("StartStopNode(): idx out of range.");
        return;
    }

    var info = _nodeMatrix[idx];
    var context = info['context'];
    var state = info['state'];
    var objLbl = document.getElementById(info['lblID']);

    if ((state == _state_unknown) || (state == _state_not_started)) {
        var msg = JSON.stringify({ msgType: "startNode", startNode: context });

        sendMessage(msg);

        EnableDisableUIControl(info['btnID'], false);

        _nodeMatrix[idx]['state'] = _state_starting;

        if (objLbl)
            objLbl.innerText = _state_starting;
    }
}
//--------------------------------------------------------------------------------------------
function setROSNodesStatus(status)
{
    var enableCalibrationBtn = true;

    // console.log(status);

    for (var i = 0; i < _nodeMatrix.length; i++) {
        var info = _nodeMatrix[i];
        var context = info['context'];
        var state = info['state'];
        var isRunning = status[context];
        var obj;
        var update = false;

        //
        // all modules need to be running for calibration
        if (!isRunning)
            enableCalibrationBtn = false;

        if ((state != _state_running) && (isRunning)) {
            _nodeMatrix[i]['state'] = state = _state_running;
            update = true;
        } else if (((state == _state_running) || (state == _state_unknown)) && (!isRunning)) {
            _nodeMatrix[i]['state'] = state = _state_not_started;
            update = true;
        }

        if (update) {
            //
            // update
            obj = document.getElementById(info['lblID']);
            if (obj) {
                console.log("changing label to '" + state + "'");
                obj.innerText = state;
            } else {
                console.log("no label object.");
            }

            if (state != _state_starting) {
                var btnID = info['btnID'];
                var btnIcon = info['btnIcon'];
    
                EnableDisableUIControl(btnID, true);

                obj = document.getElementById(btnIcon);
                if (obj) {
                    obj.className = isRunning ? "fa fa-stop" : "fa fa-play";
                }
            }
        }
    }

    if (_lblHololensSequence != null) {
        if (status.hololens_sequence)
            _lblHololensSequence.innerText = status.hololens_sequence;
        else
            _lblHololensSequence.innerText = "-";
    }

    if (_btnHoloLensCalibration != null) {
        _btnHoloLensCalibration.disabled = false; // enableCalibrationBtn ? false : true;
    }
}
