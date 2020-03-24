import weakref
import sys
import queue
import time
import threading
import logging
import json

from .rtls_util_exception import *

from rtls import RTLSManager, RTLSNode

from dataclasses import dataclass


@dataclass
class RtlsUtilLoggingLevel():
    UTIL_RAW_TRAFFIC = 60
    UTIL_TOF_RESULTS = 54
    UTIL_AOA_RESULTS = 53
    UTIL_CCI_RESULTS = 52
    UTIL_ALL = 51
    CRITICAL = 50
    ERROR = 40
    WARNING = 30
    INFO = 20
    DEBUG = 10
    ALL = 0


class RtlsUtil():
    def __init__(self, logging_file, logging_level):
        logging.basicConfig(
            filename=logging_file,
            filemode="a",
            level=logging_level,
            format='[%(asctime)s] %(filename)-18sln %(lineno)3d %(threadName)-10s %(levelname)8s - %(message)s'
        )

        self._master_node = None
        self._passive_nodes = []
        self._all_nodes = []

        self._rtls_manager = None
        self._rtls_manager_subscriber = None

        self._message_receiver_th = None
        self._message_receiver_stop = False

        self._scan_results = []
        self._scan_stopped = threading.Event()
        self._scan_stopped.clear()

        self._ble_connected = False

        self._master_disconnected = threading.Event()
        self._master_disconnected.clear()

        self._master_seed = None

        self._timeout = 30

        self.aoa_results_queue = queue.Queue()
        self.tof_results_queue = queue.Queue()
        self.conn_info_queue = queue.Queue()

        self.acc_results_queue = queue.Queue()

    def __del__(self):
        self.done()

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value

    def _rtls_wait(self, true_cond_func, nodes, timeout_message):
        timeout = time.time() + self._timeout
        timeout_reached = time.time() > timeout

        while not true_cond_func(nodes) and not timeout_reached:
            time.sleep(0.1)
            timeout_reached = time.time() > timeout

        if timeout_reached:
            raise RtlsUtilTimeoutException(f"Timeout reached while waiting for : {timeout_message}")

    def done(self):
        if self._message_receiver_th is not None:
            self._message_receiver_stop = True
            self._message_receiver_th.join()
            self._message_receiver_th = None

        if self._rtls_manager:
            self._rtls_manager.stop()
            while not self._rtls_manager.stopped:
                time.sleep(0.1)

            self._rtls_manager_subscriber = None
            self._rtls_manager = None

    def _message_receiver(self):
        while not self._message_receiver_stop:
            # Get messages from manager
            try:
                identifier, msg_pri, msg = self._rtls_manager_subscriber.pend(block=True, timeout=0.05).as_tuple()

                # Get reference to RTLSNode based on identifier in message
                sending_node = self._rtls_manager[identifier]

                if sending_node in self._passive_nodes:
                    logging.log(RtlsUtilLoggingLevel.UTIL_RAW_TRAFFIC, f"PASSIVE: {identifier} --> {msg.as_json()}")
                else:
                    logging.log(RtlsUtilLoggingLevel.UTIL_RAW_TRAFFIC, f"MASTER: {identifier} --> {msg.as_json()}")

                if msg.command == "RTLS_CMD_SCAN" and msg.type == "AsyncReq":
                    self._add_scan_result({
                        'addr': msg.payload.addr,
                        'addrType': msg.payload.addrType
                    })

                if msg.command == "RTLS_CMD_SCAN_STOP" and msg.type == "AsyncReq":
                    self._scan_stopped.set()

                if msg.command == "RTLS_CMD_CONNECT" and msg.type == "AsyncReq" and msg.payload.status == "RTLS_SUCCESS":
                    sending_node.ble_connected = True

                    if sending_node.identifier == self._master_node.identifier:
                        self._master_disconnected.clear()

                if msg.command == "RTLS_CMD_CONNECT" and msg.type == "AsyncReq" and msg.payload.status == "RTLS_LINK_TERMINATED":
                    sending_node.ble_connected = False

                    if sending_node.identifier == self._master_node.identifier:
                        self._master_disconnected.set()

                if msg.command == 'RTLS_CMD_AOA_SET_PARAMS' and msg.payload.status == 'RTLS_SUCCESS':
                    sending_node.aoa_initialized = True

                if msg.command in ["RTLS_CMD_AOA_RESULT_ANGLE",
                                   "RTLS_CMD_AOA_RESULT_RAW",
                                   "RTLS_CMD_AOA_RESULT_PAIR_ANGLES"] and msg.type == "AsyncReq":
                    self.aoa_results_queue.put({
                        "name": sending_node.name,
                        "type": "aoa",
                        "identifier": identifier,
                        "payload": msg.payload
                    })
                    logging.log(RtlsUtilLoggingLevel.UTIL_AOA_RESULTS, msg=json.dumps(msg.payload))

                if msg.command == 'RTLS_CMD_ACC_ENABLE': #and msg.payload.status == 'RTLS_SUCCESS':
                    #print(msg.payload.status)
                    print("Acc enabled")

                if msg.command == 'RTLS_CMD_ACC_RESULT_RAW' and msg.type == "AsyncReq":
                    self.acc_results_queue.put({#"Payload" : msg.payload })
                        "X0" : msg.payload.X0,
                        "X1" : msg.payload.X1,
                        "Y0" : msg.payload.Y0,
                        "Y1" : msg.payload.Y1,
                        "Z0" : msg.payload.Z0,
                        "Z1" : msg.payload.Z1
                        })
                    logging.log(RtlsUtilLoggingLevel.UTIL_ALL, msg=json.dumps(msg.payload))
                
                if msg.command == 'RTLS_CMD_ACC_RESULT' and msg.type == "AsyncReq":
                    self.acc_results_queue.put({#"Payload" : msg.payload })
                        "X" : msg.payload.X,
                        "Y" : msg.payload.Y,
                        "Z" : msg.payload.Z,
                        })
                    logging.log(RtlsUtilLoggingLevel.UTIL_ALL, msg=json.dumps(msg.payload))

                if msg.command == 'RTLS_CMD_TOF_SET_PARAMS' and msg.payload.status == 'RTLS_SUCCESS':
                    sending_node.tof_initialized = True

                if msg.command == 'RTLS_CMD_TOF_GET_SEC_SEED' and msg.payload.seed is not None:
                    self._master_seed = msg.payload.seed

                if msg.command == 'RTLS_CMD_TOF_SET_SEC_SEED' and msg.payload.status == 'RTLS_SUCCESS':
                    sending_node.seed_initialized = True

                if msg.command == 'RTLS_CMD_TOF_ENABLE' and msg.payload.status == 'RTLS_SUCCESS':
                    sending_node.tof_started = True

                if msg.command in ["RTLS_CMD_TOF_RESULT_DIST",
                                   "RTLS_CMD_TOF_RESULT_STAT",
                                   "RTLS_CMD_TOF_RESULT_RAW"] and msg.type == "AsyncReq":
                    self.tof_results_queue.put({
                        "name": sending_node.name,
                        "type": "tof",
                        "identifier": identifier,
                        "payload": msg.payload
                    })
                    logging.log(RtlsUtilLoggingLevel.UTIL_TOF_RESULTS, msg=json.dumps(msg.payload))

                if msg.command == 'RTLS_CMD_TOF_CALIBRATE' and msg.type == 'SyncRsp':
                    sending_node.tof_calibration_configured = True

                if msg.command == 'RTLS_CMD_TOF_CALIBRATE' and msg.type == 'AsyncReq':
                    sending_node.tof_calibrated = True

                if msg.command == 'RTLS_CMD_RESET_DEVICE' and msg.type == 'AsyncReq':
                    sending_node.device_resets = True

                if msg.command == 'RTLS_CMD_TOF_CALIB_NV_READ' and msg.type == 'AsyncReq':
                    sending_node.tof_calib_info = msg.payload
                    sending_node.tof_calib_read_return = True

                if msg.command == 'RTLS_CMD_CONN_INFO' and msg.type == 'SyncRsp':
                    sending_node.cci_started = True

                if msg.command == 'RTLS_EVT_CONN_INFO' and msg.type == 'AsyncReq':
                    self.conn_info_queue.put({
                        "name": sending_node.name,
                        "type": "conn_info",
                        "identifier": identifier,
                        "payload": msg.payload
                    })
                    logging.log(RtlsUtilLoggingLevel.UTIL_CCI_RESULTS, msg=json.dumps(msg.payload))

                if msg.command == 'RTLS_CMD_SET_RTLS_PARAM' and msg.payload.rtlsParamType == "RTLS_PARAM_CONNECTION_INTERVAL" and msg.payload.status == "RTLS_SUCCESS":
                    sending_node.conn_interval_updated = True

                if msg.command == 'RTLS_CMD_TOF_SWITCH_ROLE' and msg.payload.status == "RTLS_SUCCESS":
                    sending_node.role_switched = True

                if msg.command == 'RTLS_CMD_IDENTIFY' and msg.type == 'SyncRsp':
                    sending_node.identified = True
                    sending_node.identifier = msg.payload.identifier
                    sending_node.capabilities = msg.payload.capabilities
                    sending_node.devId = msg.payload.devId
                    sending_node.revNum = msg.payload.revNum


            except queue.Empty:
                pass

    def _start_message_receiver(self):
        self._message_receiver_stop = False
        self._message_receiver_th = threading.Thread(target=self._message_receiver)
        self._message_receiver_th.setDaemon(True)
        self._message_receiver_th.start()

    def _empty_queue(self, q):
        while True:
            try:
                q.get(timeout=0.5)
            except queue.Empty:
                break

    def _is_passive_in_nodes(self, nodes):
        for node in nodes:
            if not node.capabilities.get('RTLS_MASTER', False):
                return True

        return False

    ## Devices API

    def set_devices(self, devices_setting):
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Setting nodes : ".format(json.dumps(devices_setting)))
        nodes = [RTLSNode(node["com_port"], node["baud_rate"], node["name"]) for node in devices_setting]

        self._rtls_manager = RTLSManager(nodes, websocket_port=None)
        self._rtls_manager_subscriber = self._rtls_manager.create_subscriber()
        self._rtls_manager.auto_params = True

        self._start_message_receiver()
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Message receiver started")

        self._rtls_manager.start()
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "RTLS Manager started")
        time.sleep(2)
        self._master_node, self._passive_nodes, failed = self._rtls_manager.wait_identified()

        if self._master_node is None:
            raise RtlsUtilMasterNotFoundException("No one of the nodes identified as RTLS MASTER")
        # elif len(self._passive_nodes) == 0:
        #     raise RtlsUtilPassiveNotFoundException("No one of the nodes identified as RTLS PASSIVE")
        elif len(failed) > 0:
            raise RtlsUtilNodesNotIdentifiedException("{} nodes not identified at all".format(len(failed)), failed)
        else:
            pass

        self._all_nodes = [pn for pn in self._passive_nodes]  ## deep copy
        self._all_nodes.extend([self._master_node])

        for node in self._all_nodes:
            node.cci_started = False

            node.aoa_initialized = False

            node.tof_calib_info = None
            node.tof_calib_read_return = False
            node.tof_initialized = False
            node.seed_initialized = False
            node.tof_calibration_configured = False
            node.tof_calibrated = False
            node.tof_started = False

            node.ble_connected = False
            node.device_resets = False

        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Done setting node")
        return self._master_node, self._passive_nodes, self._all_nodes

    def get_devices_capability(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("nodes input must be from list type")

        for node in nodes_to_set:
            node.identified = False
            node.rtls.identify()

        true_cond_func = lambda nodes: all([n.identified for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All device to identified")

        ret = []
        for node in nodes_to_set:
            dev_info = {
                "node_mac_address": node.identifier,
                "capabilities": node.capabilities
            }

            ret.append(dev_info)

        return ret

    ######

    ## Common BLE API

    def _get_slave_by_addr(self, addr):
        for _scan_result in self._scan_results:
            if _scan_result['addr'].lower() == addr.lower():
                return _scan_result

        return None

    def _add_scan_result(self, scan_result):
        if self._get_slave_by_addr(scan_result['addr']) is None:
            self._scan_results.append(scan_result)

    def scan(self, scan_time_sec, expected_slave_bd_addr=None):
        self._scan_results = []

        timeout = time.time() + scan_time_sec
        timeout_reached = time.time() > timeout

        while not timeout_reached:
            self._scan_stopped.clear()

            self._master_node.rtls.scan()

            while not self._scan_stopped.isSet():
                time.sleep(0.1)

            if len(self._scan_results) > 0:
                if expected_slave_bd_addr is not None and self._get_slave_by_addr(expected_slave_bd_addr) is not None:
                    break

            timeout_reached = time.time() > timeout
        else:
            if len(self._scan_results) > 0:
                if expected_slave_bd_addr is not None and self._get_slave_by_addr(expected_slave_bd_addr) is not None:
                    raise RtlsUtilScanSlaveNotFoundException("Expected slave not found in scan list")
            else:
                raise RtlsUtilScanNoResultsException("No device with slave capability found")

        return self._scan_results

    @property
    def ble_connected(self):
        return self._ble_connected

    def ble_connect(self, slave, connect_interval_mSec):
        if isinstance(slave, str):
            slave = self._get_slave_by_addr(slave)
            if slave is None:
                raise RtlsUtilScanSlaveNotFoundException("Expected slave not found in scan list")
        else:
            if 'addr' not in slave.keys() or 'addrType' not in slave.keys():
                raise RtlsUtilException("Input slave not a string and not contains required keys")

        interval = int(connect_interval_mSec / 1.25)

        self._master_node.rtls.connect(slave['addrType'], slave['addr'], interval)

        true_cond_func = lambda nodes: all([n.ble_connected for n in nodes])
        self._rtls_wait(true_cond_func, self._all_nodes, "All node to connect")

        self._ble_connected = True
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Connection process done")

    def ble_disconnect(self):
        self._master_node.rtls.terminate_link()

        true_cond_func = lambda event: event.isSet()
        self._rtls_wait(true_cond_func, self._master_disconnected, "Master disconnect")

        self._ble_connected = False
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Disconnect process done")

    def set_connection_interval(self, connect_interval_mSec):
        conn_interval = int(connect_interval_mSec / 1.25)
        data_len = 2
        data_bytes = conn_interval.to_bytes(data_len, byteorder='little')

        self._master_node.conn_interval_updated = False
        self._master_node.rtls.set_rtls_param('RTLS_PARAM_CONNECTION_INTERVAL', data_len, data_bytes)

        true_cond_func = lambda nodes: all([n.conn_interval_updated for n in nodes])
        self._rtls_wait(true_cond_func, [self._master_node], "Master node set connection interval")

        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Connection Interval Updated")

    def reset_devices(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("nodes input must be from list type")

        for node in nodes_to_set:
            node.device_resets = False
            node.rtls.reset_device()

        true_cond_func = lambda nodes: all([n.device_resets for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All node to reset")

    ######

    ## CCI - Continuous Connection Info

    def cci_start(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        self._empty_queue(self.conn_info_queue)

        for node in nodes_to_set:
            node.cci_started = False
            node.rtls.get_conn_info(True)

        true_cond_func = lambda nodes: all([n.cci_started for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All node start continues connect info (CCI)")

    def cci_stop(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        for node in nodes_to_set:
            node.rtls.get_conn_info(False)

    ######

    ## ACC

    def acc_start(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        self._empty_queue(self.acc_results_queue)

        for node in nodes_to_set:
            node.acc_started = True #HÄR
        node.rtls.acc_start() #HÄR

        true_cond_func = lambda nodes: all([n.acc_started for n in nodes]) #HÄR
        self._rtls_wait(true_cond_func, nodes_to_set, "All node start ACC")

    def acc_stop(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        for node in nodes_to_set:
            node.rtls.get_conn_info(False) #HÄR

    ######

    ## AOA - Angle of Arrival
    def is_aoa_supported(self, nodes):
        devices_capab = self.get_devices_capability(nodes)
        for device_capab in devices_capab:
            if not (device_capab['capabilities'].AOA_TX == True or device_capab['capabilities'].AOA_RX == True):
                return False

        return True

    def aoa_set_params(self, aoa_params, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        try:
            for node in nodes_to_set:
                node.aoa_initialized = False
                node_role = 'AOA_MASTER' if node.capabilities.get('RTLS_MASTER', False) else 'AOA_PASSIVE'
                if str(node.devId) == "DeviceFamily_ID_CC26X0R2":
                    node.rtls.aoa_set_params(
                        node_role,
                        aoa_params['aoa_run_mode'],
                        aoa_params['aoa_cc2640r2']['aoa_cte_scan_ovs'],
                        aoa_params['aoa_cc2640r2']['aoa_cte_offset'],
                        aoa_params['aoa_cc2640r2']['aoa_cte_length']
                    )
                else:
                    node.rtls.aoa_set_params(
                        node_role,
                        aoa_params['aoa_run_mode'],
                        aoa_params['aoa_cc26x2']['aoa_slot_durations'],
                        aoa_params['aoa_cc26x2']['aoa_sample_rate'],
                        aoa_params['aoa_cc26x2']['aoa_sample_size'],
                        aoa_params['aoa_cc26x2']['aoa_sampling_control'],
                        aoa_params['aoa_cc26x2']['aoa_sampling_enable'],
                        aoa_params['aoa_cc26x2']['aoa_num_of_ant'],
                        aoa_params['aoa_cc26x2']['aoa_ant_array_switch'],
                        aoa_params['aoa_cc26x2']['aoa_ant_array']
                    )
        except KeyError as ke:
            raise RtlsUtilException("Invalid key : {}".format(str(ke)))

        true_cond_func = lambda nodes: all([n.aoa_initialized for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All node to set AOA params")

    def _aoa_set_state(self, start, cte_interval=1, cte_length=20, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        for node in nodes_to_set:
            node_role = 'AOA_MASTER' if node.capabilities.get('RTLS_MASTER', False) else 'AOA_PASSIVE'
            if str(node.devId) == "DeviceFamily_ID_CC26X0R2":
                node.rtls.aoa_start(start)
            else:
                node.rtls.aoa_start(start, cte_interval, cte_length)

    def aoa_start(self, cte_length, cte_interval, nodes=None):
        self._aoa_set_state(start=True, cte_length=cte_length, cte_interval=cte_interval, nodes=nodes)
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "AOA Started")

    def aoa_stop(self, nodes=None):
        self._aoa_set_state(start=False, nodes=nodes)
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "AOA Stopped")

    ######

    ## TOF - Time of Flight

    def is_tof_supported(self, nodes):
        devices_capab = self.get_devices_capability(nodes)
        for device_capab in devices_capab:
            if not (device_capab['capabilities'].TOF_MASTER == True or device_capab[
                'capabilities'].TOF_PASSIVE == True):
                return False

        return True

    def _tof_get_master_seed(self):
        self._master_seed = None

        self._master_node.rtls.tof_get_sec_seed()

        timeout = time.time() + self._timeout
        timeout_reached = time.time() > timeout

        while self._master_seed is None and not timeout_reached:
            time.sleep(0.1)
            timeout_reached = time.time() > timeout

        if timeout_reached:
            raise RtlsUtilTimeoutException(f"Timeout reached while waiting for : Master SEED")

    def tof_set_params(self, tof_params, seed_exchange=True, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        try:
            for node in nodes_to_set:
                node.tof_initialized = False
                node_role = 'TOF_MASTER' if node.capabilities.get('TOF_MASTER', False) else 'TOF_PASSIVE'
                node.rtls.tof_set_params(node_role,
                                         tof_params['tof_samples_per_burst'],
                                         len(tof_params['tof_freq_list']),
                                         tof_params['tof_slave_lqi_filter'],
                                         tof_params['tof_post_process_lqi_thresh'],
                                         tof_params['tof_post_process_magn_ratio'],
                                         tof_params['tof_auto_rssi'],
                                         tof_params['tof_sample_mode'],
                                         tof_params['tof_run_mode'],
                                         tof_params['tof_freq_list'])

        except KeyError as ke:
            raise RtlsUtilException("Invalid key : {}".format(str(ke)))

        true_cond_func = lambda nodes: all([n.tof_initialized for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All nodes set TOF params")

        if seed_exchange:
            self._tof_get_master_seed()

            for node in nodes_to_set:
                ## Set seed only to passive nodes
                if not node.capabilities.get('TOF_MASTER', False):
                    node.seed_initialized = False
                    node.rtls.tof_set_sec_seed(self._master_seed)

            true_cond_func = lambda nodes: all(
                [n.seed_initialized for n in nodes if not node.capabilities.get('TOF_MASTER', False)])
            self._rtls_wait(true_cond_func, nodes_to_set, "Passive nodes set SEED")

    def _tof_calib_set_passive_node(self, nodes, samples_per_freq, distance, use_nv_calib):
        for node in nodes:
            if not node.capabilities.get('TOF_MASTER', False):
                node.tof_calibration_configured = False
                node.tof_calibrated = False
                node.rtls.tof_calib(True, samples_per_freq, distance, use_nv_calib)

        true_cond_func = lambda nodes: all(
            [n.tof_calibration_configured for n in nodes if not n.capabilities.get('TOF_MASTER', False)])
        self._rtls_wait(true_cond_func, nodes, "Passive nodes to set TOF calibration params")

    def _tof_calib_set_master_node(self, nodes, samples_per_freq, distance, use_nv_calib):
        for node in nodes:
            if node.capabilities.get('TOF_MASTER', False):
                node.tof_calibration_configured = False
                node.tof_calibrated = False
                node.rtls.tof_calib(True, samples_per_freq, distance, use_nv_calib)

        true_cond_func = lambda nodes: all(
            [n.tof_calibration_configured for n in nodes if n.capabilities.get('TOF_MASTER', False)])
        self._rtls_wait(true_cond_func, nodes, "Master node to set TOF calibration params")

    def _tof_start_in_passive_node(self, nodes):
        for node in nodes:
            if not node.capabilities.get('TOF_MASTER', False):
                node.tof_started = False
                node.rtls.tof_start(True)

        true_cond_func = lambda nodes: all(
            [n.tof_started for n in nodes if not node.capabilities.get('TOF_MASTER', False)])
        self._rtls_wait(true_cond_func, nodes, "Passive nodes to start TOF")

    def _tof_start_in_master_node(self, nodes):
        for node in nodes:
            if node.capabilities.get('TOF_MASTER', False):
                node.tof_started = False
                node.rtls.tof_start(True)

        true_cond_func = lambda nodes: all([n.tof_started for n in nodes])
        self._rtls_wait(true_cond_func, nodes, "All nodes to start TOF")

    def tof_start(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("nodes input must be from list type")

        self._tof_start_in_passive_node(nodes_to_set)

        self._tof_start_in_master_node(nodes_to_set)

        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "TOF Started")

    def tof_stop(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("nodes input must be from list type")

        for node in nodes_to_set:
            node.tof_started = False
            node.rtls.tof_start(False)

        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "TOF Stopped")

    def tof_calibrate(self, samples_per_freq, distance, use_nv_calib=False, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        if self._is_passive_in_nodes(nodes_to_set):
            self._tof_calib_set_passive_node(nodes_to_set,
                                             samples_per_freq,
                                             distance,
                                             use_nv_calib)

            self._tof_start_in_passive_node(nodes_to_set)

            self._tof_calib_set_master_node(nodes_to_set,
                                            0,
                                            distance,
                                            use_nv_calib)

            self._tof_start_in_master_node(nodes_to_set)

            true_cond_func = lambda nodes: all(
                [n.tof_calibrated for n in nodes if not n.capabilities.get('TOF_MASTER', False)])
            self._rtls_wait(true_cond_func, nodes_to_set, "Passive nodes to end TOF calibration")

            for node in nodes_to_set:
                if node.capabilities.get('TOF_MASTER', False):
                    node.rtls.tof_calib(False, 0, distance, False)

        else:
            self._tof_calib_set_master_node(nodes_to_set,
                                            samples_per_freq,
                                            distance,
                                            use_nv_calib)

            self._tof_start_in_master_node(nodes_to_set)

            true_cond_func = lambda nodes: all(
                [n.tof_calibrated for n in nodes if n.capabilities.get('TOF_MASTER', False)])
            self._rtls_wait(true_cond_func, nodes_to_set, "Master nodes to end TOF calibration")

        for node in nodes_to_set:
            node.tof_started = False
            node.rtls.tof_start(False)

        self._empty_queue(self.tof_results_queue)
        logging.log(RtlsUtilLoggingLevel.UTIL_ALL, "Calibration Done")

    def tof_get_calib_info(self, nodes=None):
        nodes_to_set = self._all_nodes
        if nodes is not None:
            if isinstance(nodes, list):
                nodes_to_set = nodes
            else:
                raise RtlsUtilException("Nodes input must be from list type")

        for node in nodes_to_set:
            node.tof_calib_info = None
            node.tof_calib_read_return = False
            node.rtls.read_calib_from_NV()

        true_cond_func = lambda nodes: all([n.tof_calib_read_return for n in nodes])
        self._rtls_wait(true_cond_func, nodes_to_set, "All nodes return calibration info from NV")

        calib_info = []
        for node in nodes_to_set:
            info = {
                "node_mac_address": node.identifier,
                "calibration_info": node.tof_calib_info
            }

            calib_info.append(info)

        return calib_info

    def tof_role_switch(self, tof_master_node, tof_passive_node):
        if not tof_master_node.capabilities.get('TOF_MASTER', False):
            raise RtlsUtilException("Given TOF Master Node not active as TOF Master Node")

        if not tof_passive_node.capabilities.get('TOF_PASSIVE', False):
            raise RtlsUtilException("Given TOF Passive Node not active as TOF Passive Node")

        tof_master_node.role_switched = False
        tof_master_node.rtls.tof_switch_role("TOF_PASSIVE")

        tof_passive_node.role_switched = False
        tof_passive_node.rtls.tof_switch_role("TOF_MASTER")

        true_cond_func = lambda nodes: all([n.role_switched for n in nodes])
        self._rtls_wait(true_cond_func, [tof_master_node, tof_passive_node], "Role Switch")

    ######
