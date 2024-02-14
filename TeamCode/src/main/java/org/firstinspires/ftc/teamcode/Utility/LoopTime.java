package org.firstinspires.ftc.teamcode.Utility;

public class LoopTime
{
    double averageLoopTime = 0;
    double totalTime = 0;
    int counter = 0;
    double lastTime = 0;
    double longest = 0;
    double shortest = 10000;
    double Hz = 0;
    boolean Once = true;
    public double getLoopTime (double currentTime){
        double loopTime =  currentTime - lastTime;
        lastTime = currentTime;
        if(loopTime > longest){
            longest = loopTime;
        }
        if(loopTime < shortest){
            shortest = loopTime;
        }
        Hz = 1.0 / loopTime;
        return loopTime;
    }
    public double getAverageLoopTime (boolean start, double currentTime){
        if(start){
            if(Once) {
                counter = 0;
                totalTime = 0;
                Once = false;
            }
        }
        double loopTime =  currentTime - lastTime;
        lastTime = currentTime;
        totalTime += loopTime;
        counter++;
        averageLoopTime = totalTime/counter;
        return averageLoopTime;
    }
    public Double[] getInfo(){
        Double[] list = {longest, shortest, Hz, totalTime, averageLoopTime, Hz};
        return list;
    }
}