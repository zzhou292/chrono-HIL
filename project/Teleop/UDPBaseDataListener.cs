using UnityEngine;
using System;
using System.IO;
using System.Text;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Linq;
using System.Collections.Generic;
//this is the FoW script


public class UDPBaseDataListener : MonoBehaviour
{

    public int listenAtPort = 1209;//1209 csl, tony
    string logFilePath = "";
    string logFilePath2 = "";
    public bool listenForData = true;
    private List<Vector2[]> trackDataList = new List<Vector2[]>();     // Array to store loaded track data

    Thread receiveThread;
    UdpClient udpReceiverClient;

    // Data from Remote Simulator
    private SimulatorMessage currMsgObj = null;
    //private bool receivingData = false;

    // For Dashboard
    public float[] valArray;

    // Start is called before the first frame update
    void Start()
    {
        logFilePath = Application.persistentDataPath + "/" + "FoW_FinalData" + System.DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.fff") + ".txt";
        
        logFilePath2 = Application.persistentDataPath + "/" + "FoW_FinalData_path.txt";
        
        //string timestamp = System.DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.fff");
        //!!CHANGE WITH THE NUMBER OF LEAD VEHICLES
        string logEntry = "date time simTime latency_ms egoX egoY egoZ egoYaw egoVx egoVy egoVz egoSpeed engineRPM steerInput throttleInput brakeInput leadX leadY leadZ leadYaw leadVx leadVy leadVz leadSpeed";
        File.AppendAllText(logFilePath, logEntry + Environment.NewLine);
        LoadTrackData();
        initializeSettings();
    }

    public void initializeSettings()
    {
        valArray = new float[22];  // Updated for new data format
        startUdpThread();

    }

    public void LoadTrackData()
    {
        string directoryPath = Application.streamingAssetsPath + "/Tracks/";
        //string directoryPath = Application.dataPath + "/Resources/Tracks/";        
        File.AppendAllText(logFilePath2, directoryPath + Environment.NewLine);
        
        //string directoryPath = Application.dataPath;        
        int trackNumber = 1;

        // Debug log to indicate the start of track data loading
        Debug.Log("Loading track data...");

        while (true)
        {
            string filePath = directoryPath + "track" + trackNumber + ".txt";

            if (File.Exists(filePath))
            {
                // Read the text file
                string[] lines = File.ReadAllLines(filePath);

                // Check if the file contains any lines
                if (lines.Length == 0)
                {
                    Debug.LogError("Track file " + filePath + " is empty.");
                    trackNumber++;
                    continue;
                }

                // Convert lines to Vector2 array
                List<Vector2> trackData = new List<Vector2>();
                foreach (string line in lines)
                {
                    string[] coordinates = line.Split(new char[] {' '}, StringSplitOptions.RemoveEmptyEntries);

                    // Check if the line contains at least two coordinates
                    if (coordinates.Length < 2)
                    {
                        Debug.LogError("Insufficient number of coordinates in line of track " + trackNumber + ": " + line);
                        continue; // Skip this line
                    }

                    // Attempt to parse coordinates
                    if (float.TryParse(coordinates[0], out float x) && float.TryParse(coordinates[1], out float y))
                    {
                        trackData.Add(new Vector2(x, y));
                    }
                    else
                    {
                        Debug.LogError("Error parsing coordinates in line of track " + trackNumber + ": " + line);
                    }
                }

                // Add track data to the list
                trackDataList.Add(trackData.ToArray());

                // Debug log to indicate successful loading of track data
                Debug.Log("Track " + trackNumber + " loaded. Number of points: " + trackData.Count);

                trackNumber++;
            }
            else
            {
                // No more track files found
                break;
            }
        }

        // Debug log to indicate the end of track data loading
        Debug.Log("Track data loading complete. Total tracks loaded: " + trackDataList.Count);
    }

    public void startUdpThread()
    {

        receiveThread = new Thread(new ThreadStart(receiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

    }

    private void receiveData()
    {

        IPEndPoint controlPoint = new IPEndPoint(IPAddress.Any, listenAtPort);
        udpReceiverClient = new UdpClient(controlPoint);

        Debug.Log("UDP Receiver for Dashboard Initialized!");

        while (listenForData)
        {

            try
            {

                byte[] bData = udpReceiverClient.Receive(ref controlPoint);
                currMsgObj = processRawByteData(bData);
                //receivingData = true;

            }
            catch (Exception err)
            {

                if (!listenForData)
                {

                    Debug.Log("Not listening for dashboard data anymore.");

                }
                else
                {

                    Debug.LogError(err.ToString());

                }

            }

        }

    }

    public SimulatorMessage processRawByteData(byte[] bData)
    {

        IPEndPoint controlPoint = new IPEndPoint(IPAddress.Any, listenAtPort);
        udpReceiverClient = new UdpClient(controlPoint);
        bData = udpReceiverClient.Receive(ref controlPoint);
        SimulatorMessage toReturn = new SimulatorMessage(bData, listenForData);
        //SimulatorMessage toReturn = new SimulatorMessage(bData, listeningForChrono);
	//https://learn.microsoft.com/en-us/dotnet/api/system.bitconverter.todouble?view=net-8.0
        // Initialize an array to store the unpacked floats
        float[] floats = new float[bData.Length / sizeof(float)];
        // Iterate over the byte array and unpack floats
        for (int i = 0; i < floats.Length; i++)
        {
         // Extract 4 bytes representing a float from the byte array
            byte[] floatBytes = new byte[sizeof(float)];
            System.Array.Copy(bData, i * sizeof(float), floatBytes, 0, sizeof(float));

            // Convert the byte array to a float
            floats[i] = System.BitConverter.ToSingle(floatBytes, 0);
        }
	//Debug.Log(floats.Length);
        // Parse all 22 values from the streamer
        valArray[0] = floats[0];    // sim_time
        valArray[1] = floats[1];    // latency_condition_ms
        valArray[2] = floats[2];    // ego_x
        valArray[3] = floats[3];    // ego_y
        valArray[4] = floats[4];    // ego_z
        valArray[5] = floats[5];    // ego_yaw
        valArray[6] = floats[6];    // ego_vx
        valArray[7] = floats[7];    // ego_vy
        valArray[8] = floats[8];    // ego_vz
        valArray[9] = floats[9];    // ego_speed
        valArray[10] = floats[10];  // engine_rpm
        valArray[11] = floats[11];  // steering_input
        valArray[12] = floats[12];  // throttle_input
        valArray[13] = floats[13];  // brake_input
        valArray[14] = floats[14];  // lead_x
        valArray[15] = floats[15];  // lead_y
        valArray[16] = floats[16];  // lead_z
        valArray[17] = floats[17];  // lead_yaw
        valArray[18] = floats[18];  // lead_vx
        valArray[19] = floats[19];  // lead_vy
        valArray[20] = floats[20];  // lead_vz
        valArray[21] = floats[21];  // lead_speed
        

        // Initialize variables to store the closest point
        Vector2 closestTrackPoint = Vector2.zero;
        float minDistanceSquared = float.MaxValue; // Use squared distance for efficiency

        // Iterate through each track in trackDataList
        foreach (Vector2[] trackData in trackDataList)
        {
            // Iterate through each point in the track
            foreach (Vector2 trackPoint in trackData)
            {
                // Calculate the squared distance to the ego vehicle's location
                float dx = trackPoint.x - valArray[2]; // Difference in x coordinates (ego_x)
                float dy = trackPoint.y - valArray[3]; // Difference in y coordinates (ego_y)
                float distanceSquared = dx * dx + dy * dy; // Squared distance

                // Check if this point is closer than the current closest point
                if (distanceSquared < minDistanceSquared)
                {
                    minDistanceSquared = distanceSquared;
                    closestTrackPoint = trackPoint;
                }
            }
        }

        // Now, closestTrackPoint contains the closest point on the track to the ego vehicle's location

        // Calculate the deviation from the lane center
        float deviationFromCenter = closestTrackPoint.x - valArray[2]; // Subtract x-coordinate of closest point from ego vehicle's x-coordinate

        // Log the deviation from the lane center
        //valArray[8] = deviationFromCenter;   // LaneDeviation
        //valArray[9] = closestTrackPoint.x;   // LaneDeviation
        //valArray[10] = closestTrackPoint.y;   // LaneDeviation
          
        // Log all values
        string timestamp = System.DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss.fff");
        string logEntry = timestamp;
        for (int i = 0; i < 22; i++)
        {
            logEntry += " " + valArray[i];
        }
        File.AppendAllText(logFilePath, logEntry + Environment.NewLine);
        return toReturn;

    }

    void OnApplicationQuit()
    {

        try
        {
            listenForData = false;
            udpReceiverClient.Close();

        }
        catch (Exception err)
        {

            Debug.Log("Dashboard is closing.");
            Debug.LogError(err.ToString());

        }

    }

}
