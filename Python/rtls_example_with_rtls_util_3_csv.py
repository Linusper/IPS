import os
import sys
import time
import json
import queue
import threading
import datetime

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
    csv_fieldnames_acc = ['X', 'Y', 'Z']
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

    while True:
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

                if sample.name== "CC2640r2 Passive 1":
                    csv_list1.append(sample)

                elif sample.name == "CC2640r2 Passive 2":
                    csv_list2.append(sample)
                #if len(csv_list1) == len(csv_list2):


            elif isinstance(data, str) and data == "STOP":
                print("STOP Command Received")

                if len(csv_list1):
                    for sample_row in csv_list1:
                        csv_writer.writerow(sample_row._asdict())
                if len(csv_list2):
                    for sample_row in csv_list2:
                        csv_writer.writerow(sample_row._asdict())

                if len(csv_list_rssi):
                    for sample_row_rssi in csv_list_rssi:
                        csv_writer_rssi.writerow(sample_row_rssi._asdict())

                break
            else:
                pass
        except queue.Empty:
            continue

def results_parsing_acc(q):
    global csv_writer_acc
    global csv_row_acc
    csv_list_acc = []

    while True:
        try:
            data = q.get(block=True, timeout=0.5)
            if isinstance(data, dict):
                data_time = datetime.datetime.now().strftime("[%m:%d:%Y %H:%M:%S:%f] :")

                sample_acc = csv_row_acc( X=data['payload']['X'],
                Y=data['payload']['Y'],
                Z=data['payload']['Z'])

                print(sample_acc)
                csv_list_acc.append(sample_acc)

            if isinstance(data, str) and data == "STOP":
                print("STOP Command Received")

                if len(csv_list_acc):
                    for sample_row_acc in csv_list_acc:
                        csv_writer_acc.writerow(sample_row_acc._asdict())
                break
            else:
                pass
        except queue.Empty:
            continue

def results_parsing_rssi(q):
    global csv_writer_rssi
    global csv_row_rssi
    csv_list_rssi = []

    while True:
        try:
            data = q.get(block=True, timeout=0.5)
            if isinstance(data, dict):
                data_time = datetime.datetime.now().strftime("[%m:%d:%Y %H:%M:%S:%f] :")

                sample_rssi = csv_row_rssi(rssi = ['payload']['rssi'])

                print(sample_rssi)
                csv_list_rssi.append(sample_rssi)

            elif isinstance(data, str) and data == "STOP":
                print("STOP Command Received")

                if len(csv_list_rssi):
                    for sample_row_rssi in csv_list_rssi:
                        csv_writer_rssi.writerow(sample_row_rssi._asdict())

                break
            else:
                pass
        except queue.Empty:
            continue


## Main Function
def main():
    ## Predefined parameters
    slave_bd_addr = None  # "80:6F:B0:1E:38:C3" # "54:6C:0E:83:45:D8"
    scan_time_sec = 15
    connect_interval_mSec = 100

    ## Continuous Connection Info Demo Enable / Disable
    cci = False
    ## Accelerometer Enable / Disable
    acc = True
    ## RSSI Enable/ Disable
    rssi = True
    ## Angle of Arival Demo Enable / Disable
    aoa = False
    ## Time of Flight Demo Enable / Disable
    tof = False
    tof_use_calibrate_from_nv = False
    ## Switch TOF Role Demo Enable / Disable
    tof_switch_role = False
    ## Update connection interval on the fly Demo Enable / Disable
    update_conn_interval = False
    new_connect_interval_mSec = 80

    ## Taking python file and replacing extension from py into log for output logs + adding data time stamp to file
    data_time = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    logging_file = f"{data_time}_{os.path.basename(__file__).replace('.py', '.log')}"
    ## Initialize RTLS Util instance
    rtlsUtil = RtlsUtil(logging_file, RtlsUtilLoggingLevel.UTIL_ALL)
    ## Update general time out for all action at RTLS Util [Default timeout : 30 sec]
    rtlsUtil.timeout = 30

    all_nodes = []
    try:
        devices = [
            {"com_port": "COM3", "baud_rate": 460800, "name": "CC26x2 Master"},
            # {"com_port": "COM29", "baud_rate": 460800, "name": "CC26x2 Passive"},
            # {"com_port": "COM23", "baud_rate": 460800, "name": "CC2640r2 TOF Master"},
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

        if rssi:
            ## Setup thread to pull out received data from devices on screen
            th_acc_parsing = threading.Thread(target=results_parsing_rssi, args=(rtlsUtil.rssi_results_queue,))
            th_acc_parsing.setDaemon(True)
            th_acc_parsing.start()
            print("RSSI Callback Set")

            rtlsUtil.rssi_start()
            print("rssi Started")

        ## Start angle of arrival feature
        if aoa:
            if rtlsUtil.is_aoa_supported(all_nodes):
                aoa_params = {
                    "aoa_run_mode": "AOA_MODE_ANGLE",  ## AOA_MODE_ANGLE, AOA_MODE_PAIR_ANGLES, AOA_MODE_RAW
                    "aoa_cc2640r2": {
                        "aoa_cte_scan_ovs": 4,
                        "aoa_cte_offset": 4,
                        "aoa_cte_length": 20
                    },
                    "aoa_cc26x2": {
                        "aoa_slot_durations": 1,
                        "aoa_sample_rate": 1,
                        "aoa_sample_size": 1,
                        "aoa_sampling_control": 0,
                        "aoa_sampling_enable": 1,
                        "aoa_num_of_ant": 3,
                        "aoa_ant_array_switch": 27,
                        "aoa_ant_array": [28, 29, 30]
                    }
                }
                rtlsUtil.aoa_set_params(aoa_params)
                print("AOA Paramas Set")

                ## Setup thread to pull out received data from devices on screen
                th_aoa_results_parsing = threading.Thread(target=results_parsing_aoa, args=(rtlsUtil.aoa_results_queue,))
                th_aoa_results_parsing.setDaemon(True)
                th_aoa_results_parsing.start()
                print("AOA Callback Set")

                rtlsUtil.aoa_start(cte_length=20, cte_interval=1)
                print("AOA Started")
            else:
                print("=== Warring ! One of the devices does not support AoA functionality ===")
        '''
        ## Start time of flight feature
        if tof:
            if rtlsUtil.is_tof_supported(all_nodes):
                tof_params = {
                    "tof_sample_mode": "TOF_MODE_DIST",  ## TOF_MODE_DIST, TOF_MODE_STAT, TOF_MODE_RAW
                    "tof_run_mode": "TOF_MODE_CONT",
                    "tof_slave_lqi_filter": 255,
                    "tof_post_process_lqi_thresh": 255,
                    "tof_post_process_magn_ratio": 111,
                    "tof_samples_per_burst": 256,
                    "tof_freq_list": [2408, 2412, 2418, 2424],  ## [2408, 2412, 2414, 2418, 2420, 2424]
                    "tof_auto_rssi": -55,
                }
                rtlsUtil.tof_set_params(tof_params)
                print("TOF Paramas + Seed Set")

                ## Code below demonstrate option where the user doesn't want to use internal calibration
                if tof_params['tof_sample_mode'] == "TOF_MODE_DIST":
                    rtlsUtil.tof_calibrate(samples_per_freq=1000, distance=1, use_nv_calib=tof_use_calibrate_from_nv)
                    print("Calibration Done")

                    # print(json.dumps(rtlsUtil.tof_get_calib_info(), indent=4))
                    # print("Calibration Info Done")

                ## Setup thread to pull out received data from devices on screen
                th_tof_results_parsing = threading.Thread(target=results_parsing_aoa, args=(rtlsUtil.tof_results_queue,))
                th_tof_results_parsing.setDaemon(True)
                th_tof_results_parsing.start()
                print("TOF Callback Set")

                rtlsUtil.tof_start()
                print("TOF Started")

                ## Start switch role feature while TOF is running
                if tof_switch_role and len(passive_nodes) > 0:
                    time.sleep(2)
                    print("Slept for 2 sec before switching roles")

                    rtlsUtil.tof_stop()
                    print("TOF Stopped")

                    master_capab = rtlsUtil.get_devices_capability(nodes=[master_node])[0]
                    print(f"RTLS MASTER capability before role switch: {json.dumps(master_capab, indent=4)}")

                    rtlsUtil.tof_role_switch(tof_master_node=master_node, tof_passive_node=passive_nodes[0])
                    print("TOF Role Switch Done")

                    master_capab = rtlsUtil.get_devices_capability(nodes=[master_node])[0]
                    print(f"RTLS MASTER capability after role switch: {json.dumps(master_capab, indent=4)}")

                    rtlsUtil.tof_calibrate(samples_per_freq=1000, distance=1)
                    print("Calibration Done")

                    rtlsUtil.tof_start()
                    print("TOF Re-Started")
            else:
                print("=== Warring ! One of the devices does not support ToF functionality ===")
            '''

        ## Update connection interval after connection is set
        if update_conn_interval:
            time.sleep(2)
            print("Sleep for 2 sec before update connection interval")

            rtlsUtil.set_connection_interval(new_connect_interval_mSec)
            print(f"Update Connection Interval into : {new_connect_interval_mSec} mSec")

        ## Sleep code to see in the screen receives data from devices
        timeout_sec = 30
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

        if rssi:
            rtlsUtil.acc_results_queue.put("STOP")
            print("Try to stop RSSI result parsing thread")

            #print("CCI Stopped")
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
            rtlsUtil.ble_disconnect()
            print("Master Disconnected")

        rtlsUtil.done()
        print("Done")

        rtlsUtil = None


if __name__ == '__main__':
    main()
