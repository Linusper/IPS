import os
import sys
import time
import json
import queue
import threading
import datetime
import csv
from collections import namedtuple

## Uncomment line below for local debug of packages
# sys.path.append(r"..\unpi")
# sys.path.append(r"..\rtls")
# sys.path.append(r"..\rtls_util")

from rtls_util import RtlsUtil, RtlsUtilLoggingLevel, RtlsUtilException, RtlsUtilTimeoutException, \
    RtlsUtilNodesNotIdentifiedException, RtlsUtilScanNoResultsException

csv_writer = None
csv_row = None
csv_writer_acc = None
csv_row_acc = None
csv_writer_rssi = None
csv_row_rssi = None

nrThreads = 0
nrThreadsRunning = 0

def initialize_csv_file():
    global csv_writer
    global csv_row
    global csv_writer_acc
    global csv_row_acc
    global csv_writer_rssi
    global csv_row_rssi

    # Prepare csv file to save data
    data_time = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    filename = f"{data_time}_rtls_angle_samples.csv"
    outfile = open(filename, 'w', newline='')
    filename_acc = f"{data_time}_rtls_acc_samples.csv"
    outfile_acc = open(filename_acc, 'w', newline='')
    filename_rssi = f"{data_time}_rtls_rssi_samples.csv"
    outfile_rssi = open(filename_rssi, 'w', newline='')

    csv_fieldnames = ['name', 'antenna', 'rssi', 'angle']
    csv_row = namedtuple('csv_row', csv_fieldnames)
    csv_fieldnames_acc = ['X', 'Y', 'Z']#,'dt']
    csv_row_acc = namedtuple('csv_row_acc', csv_fieldnames_acc)
    csv_fieldnames_rssi = ['rssi']
    csv_row_rssi = namedtuple('csv_row_rssi', csv_fieldnames_rssi)

    csv_writer = csv.DictWriter(outfile, fieldnames=csv_fieldnames)
    csv_writer.writeheader()
    csv_writer_acc = csv.DictWriter(outfile_acc, fieldnames=csv_fieldnames_acc)
    csv_writer_acc.writeheader()
    csv_writer_rssi = csv.DictWriter(outfile_rssi, fieldnames=csv_fieldnames_rssi)
    csv_writer_rssi.writeheader()

## User function to proces
def results_parsing_aoa(q):
    global csv_writer
    global csv_row
    csv_list1 = []
    csv_list2 = []

    global nrThreads
    global nrThreadsRunning

    nrThreadsRunning = nrThreadsRunning + 1
    while True:
        if nrThreadsRunning == nrThreads:
            try:
                data = q.get(block=True, timeout=0.5)
                if isinstance(data, dict):
                    data_time = datetime.datetime.now().strftime("[%m:%d:%Y %H:%M:%S:%f] :")
                    #print(f"{data_time} {json.dumps(data)}")

                    sample = csv_row(name=data['name'], 
                        antenna=data['payload']['antenna'], 
                        rssi=data['payload']['rssi'],
                        angle=data['payload']['angle'])

                    print(sample)

                    if sample.name == "CC2640r2 Passive V":
                        csv_list1.append(sample)

                    elif sample.name == "CC2640r2 Passive H":
                        csv_list2.append(sample)
                    #if len(csv_list1) == len(csv_list2):


                elif isinstance(data, str) and data == "STOP":
                    print("STOP Command Received for AOA")

                    if len(csv_list1):
                        for sample_row in csv_list1:
                            csv_writer.writerow(sample_row._asdict())
                    if len(csv_list2):
                        for sample_row in csv_list2:
                            csv_writer.writerow(sample_row._asdict())

                    break
                else:
                    pass
            except queue.Empty:
                continue
        else:
            try:
                q.get(timeout=0.5)
            except queue.Empty:
                continue

def results_parsing_acc(q):
    global csv_writer_acc
    global csv_row_acc
    csv_list_acc = []

    global nrThreads
    global nrThreadsRunning

    nrThreadsRunning = nrThreadsRunning + 1
    while True:
        if nrThreadsRunning == nrThreads:
            try:
                data = q.get(block=True, timeout=0.5)
                if isinstance(data, dict):
                    data_time = datetime.datetime.now().strftime("[%m:%d:%Y %H:%M:%S:%f] :")

                    sample_acc = csv_row_acc( X=data['X'],
                    Y=data['Y'],
                    Z=data['Z'],)
                   # dt=data['dt'])

                    print(sample_acc)
                    csv_list_acc.append(sample_acc)

                if isinstance(data, str) and data == "STOP":
                    print("STOP Command Received for ACC")

                    if len(csv_list_acc):
                        for sample_row_acc in csv_list_acc:
                            csv_writer_acc.writerow(sample_row_acc._asdict())
                    break
                else:
                    pass
            except queue.Empty:
                continue
        else:
            try:
                q.get(timeout=0.5)
            except queue.Empty:
                continue

def results_parsing_rssi(q):
    global csv_writer_rssi
    global csv_row_rssi
    csv_list_rssi = []

    global nrThreads
    global nrThreadsRunning

    nrThreadsRunning = nrThreadsRunning + 1
    while True:
        if nrThreadsRunning == nrThreads:
            try:
                data = q.get(block=True, timeout=0.5)
                if isinstance(data, dict):
                    data_time = datetime.datetime.now().strftime("[%m:%d:%Y %H:%M:%S:%f] :")

                    sample_rssi = csv_row_rssi(rssi = data['rssi'])

                    print(sample_rssi)
                    csv_list_rssi.append(sample_rssi)

                elif isinstance(data, str) and data == "STOP":
                    print("STOP Command Received for RSSI")

                    if len(csv_list_rssi):
                        for sample_row_rssi in csv_list_rssi:
                            csv_writer_rssi.writerow(sample_row_rssi._asdict())

                    break
                else:
                    pass
            except queue.Empty:
                continue
        else:
            try:
                q.get(timeout=0.5)
            except queue.Empty:
                continue 


## Main Function
def main():
    global nrThreads
    initialize_csv_file()
    ## Predefined parameters
    slave_bd_addr = None  # "80:6F:B0:1E:38:C3" # "54:6C:0E:83:45:D8"
    scan_time_sec = 15
    connect_interval_mSec = 100

    ## Continuous Connection Info Demo Enable / Disable
    cci = False
    ## Accelerometer Enable / Disable
    acc = True
    ## Accelerometer Enable / Disable
    acc_raw = False
    ## RSSI Enable/ Disable
    rssi = True
    ## Angle of Arival Demo Enable / Disable
    aoa = True
    ## Time of Flight Demo Enable / Disable
    tof = False
    tof_use_calibrate_from_nv = False
    ## Switch TOF Role Demo Enable / Disable
    tof_switch_role = False
    ## Update connection interval on the fly Demo Enable / Disable
    update_conn_interval = False
    new_connect_interval_mSec = 80

    nrThreads = cci + acc + acc_raw + rssi + aoa

    ## Taking python file and replacing extension from py into log for output logs + adding data time stamp to file
    data_time = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    logging_file_path = os.path.join(os.path.curdir, os.path.basename(__file__).replace('.py', '_log'))
    if not os.path.isdir(logging_file_path):
        os.makedirs(logging_file_path)
    logging_file = os.path.join(logging_file_path, f"{data_time}_{os.path.basename(__file__).replace('.py', '.log')}")

    ## Initialize RTLS Util instance
    rtlsUtil = RtlsUtil(logging_file, RtlsUtilLoggingLevel.INFO)
    ## Update general time out for all action at RTLS Util [Default timeout : 30 sec]
    rtlsUtil.timeout = 30

    all_nodes = []
    try:
        devices = [
            {"com_port": "COM3", "baud_rate": 460800, "name": "CC2640r2 Master"},
            {"com_port": "COM8", "baud_rate": 460800, "name": "CC2640r2 Passive V"},
            {"com_port": "COM9", "baud_rate": 460800, "name": "CC2640r2 Passive H"},
            # {"com_port": "COM21", "baud_rate": 460800, "name": "CC2640r2 TOF Passive"}
        ]
        ## Setup devices
        master_node, passive_nodes, all_nodes = rtlsUtil.set_devices(devices)
        print(f"Master : {master_node} \nPassives : {passive_nodes} \nAll : {all_nodes}")

        ## Reset devices for initial state of devices
        rtlsUtil.reset_devices()
        print("Devices Reset")

        ## Code below demonstrates two option of scan and connect
        ## 1. Then user know which slave to connect
        ## 2. Then user doesn't mind witch slave to use
        if slave_bd_addr is not None:
            print(f"Start scan of {slave_bd_addr} for {scan_time_sec} sec")
            scan_results = rtlsUtil.scan(scan_time_sec, slave_bd_addr)
            print(f"Scan Results: {scan_results}")

            rtlsUtil.ble_connect(slave_bd_addr, connect_interval_mSec)
            print("Connection Success")
        else:
            print(f"Start scan for {scan_time_sec} sec")
            scan_results = rtlsUtil.scan(scan_time_sec)
            print(f"Scan Results: {scan_results}")

            rtlsUtil.ble_connect(scan_results[0], connect_interval_mSec)
            print("Connection Success")

        ## Start continues connection info feature
        if cci:
            ## Setup thread to pull out received data from devices on screen
            th_cci_parsing = threading.Thread(target=results_parsing_aoa, args=(rtlsUtil.conn_info_queue,))
            th_cci_parsing.setDaemon(True)
            th_cci_parsing.start()
            print("CCI Callback Set")

            rtlsUtil.cci_start()
            print("CCI Started")
        if acc:
            ## Setup thread to pull out received data from devices on screen
            th_acc_parsing = threading.Thread(target=results_parsing_acc, args=(rtlsUtil.acc_results_queue,))
            th_acc_parsing.setDaemon(True)
            th_acc_parsing.start()
            print("ACC Callback Set")

            rtlsUtil.acc_start()
            print("ACC Started")

        if acc_raw:
            ## Setup thread to pull out received data from devices on screen
            th_acc_parsing = threading.Thread(target=results_parsing_acc, args=(rtlsUtil.acc_results_raw_queue,))
            th_acc_parsing.setDaemon(True)
            th_acc_parsing.start()
            print("ACC Callback Set")

            rtlsUtil.acc_start()
            print("ACC Started")

        if rssi:
            ## Setup thread to pull out received data from devices on screen
            th_rssi_parsing = threading.Thread(target=results_parsing_rssi, args=(rtlsUtil.rssi_results_queue,))
            th_rssi_parsing.setDaemon(True)
            th_rssi_parsing.start()
            print("RSSI Callback Set")

            rtlsUtil.rssi_start()
            print("RSSI Started")

        ## Start angle of arrival feature
        if aoa:
            if rtlsUtil.is_aoa_supported(all_nodes):
                aoa_params = {
                    "aoa_run_mode": "AOA_MODE_ANGLE",  ## AOA_MODE_ANGLE, AOA_MODE_PAIR_ANGLES, AOA_MODE_RAW
                    "aoa_cc2640r2": {
                        "aoa_cte_scan_ovs": 4,
                        "aoa_cte_offset": 4,
                        "aoa_cte_length": 20,
                        "aoa_sampling_control": int('0x00', 16),
                        ## bit 0   - 0x00 - default filtering, 0x01 - RAW_RF no filtering - not supported,
                        ## bit 4,5 - 0x00 - default both antennas, 0x10 - ONLY_ANT_1, 0x20 - ONLY_ANT_2
                    },
                    "aoa_cc26x2": {
                        "aoa_slot_durations": 1,
                        "aoa_sample_rate": 1,
                        "aoa_sample_size": 1,
                        "aoa_sampling_control": int('0x10', 16),
                        ## bit 0   - 0x00 - default filtering, 0x01 - RAW_RF no filtering,
                        ## bit 4,5 - default: 0x10 - ONLY_ANT_1, optional: 0x20 - ONLY_ANT_2
                        "aoa_sampling_enable": 1,
                        "aoa_pattern_len": 3,
                        "aoa_ant_pattern": [0, 1, 2]
                    }
                }
                rtlsUtil.aoa_set_params(aoa_params)
                print("AOA Params Set")

                ## Setup thread to pull out received data from devices on screen
                th_aoa_results_parsing = threading.Thread(target=results_parsing_aoa, args=(rtlsUtil.aoa_results_queue,))
                th_aoa_results_parsing.setDaemon(True)
                th_aoa_results_parsing.start()
                print("AOA Callback Set")

                rtlsUtil.aoa_start(cte_length=20, cte_interval=1)
                print("AOA Started")
            else:
                print("=== Warning ! One of the devices does not support AoA functionality ===")

        ## Update connection interval after connection is set
        if update_conn_interval:
            time.sleep(2)
            print("Sleep for 2 sec before update connection interval")

            rtlsUtil.set_connection_interval(new_connect_interval_mSec)
            print(f"Update Connection Interval into : {new_connect_interval_mSec} mSec")

        ## Sleep code to see in the screen receives data from devices
        timeout_sec = 60
        print("Going to sleep for {} sec".format(timeout_sec))
        timeout = time.time() + timeout_sec
        while timeout >= time.time():
            time.sleep(0.1)

    except RtlsUtilNodesNotIdentifiedException as ex:
        print(f"=== ERROR: {ex} ===")
        print(ex.not_indentified_nodes)
    except RtlsUtilTimeoutException as ex:
        print(f"=== ERROR: {ex} ===")
    except RtlsUtilException as ex:
        print(f"=== ERROR: {ex} ===")
    finally:
        if cci:
            rtlsUtil.conn_info_queue.put("STOP")
            print("Try to stop CCI result parsing thread")

            rtlsUtil.cci_stop()
            print("CCI Stopped")

        if acc:
            rtlsUtil.acc_results_queue.put("STOP")
            print("Try to stop ACC result parsing thread")

            rtlsUtil.acc_stop()
            print("ACC Stopped")

        if rssi:
            rtlsUtil.rssi_results_queue.put("STOP")
            print("Try to stop RSSI result parsing thread")

            rtlsUtil.rssi_stop()
            print("RSSI Stopped")

        if aoa and rtlsUtil.is_aoa_supported(all_nodes):
            rtlsUtil.aoa_results_queue.put("STOP")
            print("Try to stop AOA result parsing thread")

            rtlsUtil.aoa_stop()
            print("AOA Stopped")

        if tof and rtlsUtil.is_tof_supported(all_nodes):
            rtlsUtil.tof_results_queue.put("STOP")
            print("Try to stop TOF result parsing thread")

            rtlsUtil.tof_stop()
            print("TOF Stopped")

        if rtlsUtil.ble_connected:
            try:
                rtlsUtil.ble_disconnect()
                print("Master Disconnected")
            except RtlsUtilTimeoutException:
                print("Master could not disconnect")

        rtlsUtil.done()
        print("Done")

        rtlsUtil = None


if __name__ == '__main__':
    main()
