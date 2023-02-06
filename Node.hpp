#pragma once 

class Node{
    public:
        Node();
        Node(int poseX, int poseY, int currTime, double* map, int xSize, int ySize);
        
        int GetPoseX() const;
        int GetPoseY() const;
        int GetCurrentTime() const;

        double GetCellCost() const;
        double GetGValue() const;
        void SetGValue(const double val);
        double GetHeuristics() const;
        void SetHeuristics(const double val);

        bool GetBoolClosed() const; 
        void SetBoolClosed(const bool val);
        bool GetBoolExpanded() const;
        void SetBoolExpanded(const bool val);

    private:
        // a 3D state of mx, my, mt
        int mX;
        int mY;
        int mT;
        // Cell cost g-value and heuristics
        double mC; // might not need this
        double mG;
        double mH;
        // Bool Closed
        bool mbClosed;
        bool mbExpanded;
 
};