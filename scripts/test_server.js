#!/usr/bin/env node

'use strict';

const API_VERSION = 0;

const express = require('express');

var request = require('request');

const rosnodejs = require('rosnodejs');
const ifttt_msgs = rosnodejs.require('ifttt').msg;

const MAKER_KEY_PARAM = "/maker_key";
const TRIGGER_NAMES_PARAM = "/trigger_names";
const ACTION_SPECS_PARAM = "/action_specs";

const ifttt_app_server = express();
var http_server = null;

/*
 * To be passed to express.json built-in middleware for parsing json request bodies
 */
function actionGoalReviver(key, value) {
  if (key === 'options') {
    // ignore options for the meantime, because action support is only experimental in rosnodejs
    rosnodejs.log.warn('Server: "options" are ignored in action requests');
    return undefined;
  } else if(value.hasOwnProperty('type') && value.hasOwnProperty('value')) {
    // the type and value of a field in the action goal
    // TODO: potentially validate value against definition of field type
    // delete type property from final object, which should be used directly as a goal object
    return value.value;
  } else if(key.length == 0) {
    // when key is an empty string, value is the entire object parsed from JSON
    // return a goal object with all parsed fields
    return value.args
  } else {
    // must return value or else the property will be deleted (e.g. type, value for each field, or fields in nested messages)
    // TODO: consider how user should specify nested messages e.g. whether or not to include 'type' properties
    return value;
  }
};

function fetchParams(nodeHandle) {
  var fetchMakerKey = nodeHandle.hasParam(MAKER_KEY_PARAM)
    .then(exists => {
      if (exists) {
        return nodeHandle.getParam(MAKER_KEY_PARAM);
      } else {
        return Promise.reject(`Parameter ${MAKER_KEY_PARAM} is required`);
      }
    });

  var fetchTriggerNames = nodeHandle.getParam(TRIGGER_NAMES_PARAM);
  var fetchActionSpecs = nodeHandle.getParam(ACTION_SPECS_PARAM);

  return Promise.all([fetchMakerKey, fetchTriggerNames, fetchActionSpecs, nodeHandle]);
};

function TriggerHandler(makerKey, triggerName, nodeHandle) {
  this.subscriber = nodeHandle.subscribe(triggerName, ifttt_msgs.TriggerEvent,
    trigger => {
      rosnodejs.log.info(`Trigger [${triggerName}]: Firing`);
      request(
        {
          method: 'POST',
          uri: `https://maker.ifttt.com/trigger/${triggerName}/with/key/${makerKey}`
        },
        (error, response, body) => {
          if (response.statusCode != 200) {
            rosnodejs.log.error(`Trigger [${triggerName}]: Error [${response.statusCode} ${body}]`);
          }
        }
      );
    });
}

function constructTriggerHandlers(makerKey, triggerNames, nodeHandle) {
  var handlers = new Array();
  triggerNames.forEach(triggerName => {
    handlers.push(new TriggerHandler(makerKey, triggerName, nodeHandle));
  });
  return Promise.resolve(handlers);
}

function ActionHandler(actionName, actionType, nodeHandle) {
  this.actionClient = nodeHandle.actionClientInterface(actionName, actionType);
  // Lambda uses this from enclosing context
  this.handleActionRequest = (req, res, next) => {
    this.actionClient.sendGoal({goal: req.body});
    res.status(200).json({});
  }
}

function constructActionHandlers(actionSpecs, nodeHandle) {
  var handlers = new Array();
  Object.getOwnPropertyNames(actionSpecs).forEach(name => {
    var handler = new ActionHandler(name, actionSpecs[name], nodeHandle);
    ifttt_app_server.post(`/ifttt-ros/v${API_VERSION}/actions/:${name}`,
                          [express.json({reviver: actionGoalReviver}), handler.handleActionRequest]);
    handlers.push(handler);
  });
  return Promise.resolve(handlers);
}

function constructAllHandlers([makerKey, triggerNames, actionSpecs, nodeHandle]) {
  var triggerHandlers = constructTriggerHandlers(makerKey, triggerNames, nodeHandle);
  var actionHandlers = constructActionHandlers(actionSpecs, nodeHandle);
  return Promise.all([triggerHandlers, actionHandlers]);
}

function testServer() {
  rosnodejs.initNode('/test_server').then(fetchParams).then(constructAllHandlers)
    .then(([triggerHandlers, actionHandlers]) => {
      rosnodejs.log.info(`Server: Triggers [${triggerHandlers.length}]`);
      rosnodejs.log.info(`Server: Actions [${actionHandlers.length}]`);
      http_server = ifttt_app_server.listen(8080);
    })
    .catch(reason => {
      rosnodejs.log.error(`Server: Error [${reason}]`);
      if (reason instanceof Error) {
        rosnodejs.log.error(`Server: Trace [${reason.stack}]`);
      }
      if(http_server != null) {
        http_server.close()
      }
      rosnodejs.shutdown();
    });
}

if (require.main === module) {
  testServer();
}