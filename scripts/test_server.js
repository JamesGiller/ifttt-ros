#!/usr/bin/env node

'use strict';

var request = require('request');

const rosnodejs = require('rosnodejs');
const ifttt_msgs = rosnodejs.require('ifttt').msg;

const MAKER_KEY_PARAM = "/maker_key";
const TRIGGER_NAMES_PARAM = "/trigger_names";

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

  return Promise.all([fetchMakerKey, fetchTriggerNames, nodeHandle]);
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

function constructTriggerHandlers([makerKey, triggerNames, nodeHandle]) {
  var handlers = new Array();
  triggerNames.forEach(triggerName => {
    handlers.push(new TriggerHandler(makerKey, triggerName, nodeHandle));
  });
  return Promise.resolve(handlers);
}

function testServer() {
  rosnodejs.initNode('/test_server').then(fetchParams).then(constructTriggerHandlers)
    .then(triggerHandlers => {
      rosnodejs.log.info(`Server: Triggers [${triggerHandlers.length}]`);
    })
    .catch(reason => {
      rosnodejs.log.error(`Server: Error [${reason}]`);
      if (reason instanceof Error) {
        rosnodejs.log.error(`Server: Trace [${reason.stack}]`);
      }
      rosnodejs.shutdown();
    });
}

if (require.main === module) {
  testServer();
}