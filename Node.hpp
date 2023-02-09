#pragma once

class Node{
    public:
        Node();
        Node(int poseX, int poseY, int currTime);
        
        int GetPoseX() const;
        int GetPoseY() const;
        int GetCurrentTime() const;
        void SetCurrentTime(const int val);

        double GetGValue() const;
        void SetGValue(const double val);
        double GetHeuristics() const;
        void SetHeuristics(const double val);
        double GetFValue() const;
        void SetFValue(const double val);
        
        Node* GetParent() const;
        void SetParent(Node* pparent);
        

    private:
        // a 3D state of mx, my, mt
        int mX;
        int mY;
        int mT;
        // Cell cost g-value and heuristics
        double mG;
        double mH;
        double mF;
        // Backtracking 
        Node* mpparent;
 
};