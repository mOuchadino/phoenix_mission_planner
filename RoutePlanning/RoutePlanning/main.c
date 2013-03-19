//
//  main.c
//  RoutePlanning
//
//  Created by Julian Tatsch on 12.03.13.
//  Copyright (c) 2013 Julian Tatsch. All rights reserved.
//

#include <stdio.h>
#include <math.h>
#include <stdbool.h>


const int maxRows =10;
const int maxCols=10;
const int goalX=7;
const int goalY=7;

enum fieldTypes {FREE=1, OBSTACLE=1000, START=0, GOAL=0};

struct Node
{
    int totalCost;
    int wayCost;
    int x;
    int y;
    bool closed;
    struct Node * parent;
};

struct List {
    struct Node * node;
    struct List * next;
} openList;

struct Node map[maxRows][maxCols];

void enqueueInSortedList(struct Node * node){
    struct List newElement;
    newElement.node=node; 
    newElement.next=NULL;
    if (!openList.node) {
        openList=newElement;
    } else {
        struct List currentListElement=openList;
        if(node->totalCost<currentListElement.node->totalCost){
            newElement.next=&openList;
            openList=newElement;
        } else {
            while (currentListElement.node->totalCost<=node->totalCost) {
                if (!currentListElement.next) {
                    break;
                } else {
                    currentListElement=*currentListElement.next;
                }
            }
            openList.next=&newElement;
        }
    }
}

struct Node * dequeueNodeFromOpenList(){
    struct Node * dequeuedNode;
    if (openList.node) {
        dequeuedNode=openList.node;
        if (openList.next!=NULL) {
            openList=*openList.next;
        } else {
            openList.node=NULL;
        }
        return dequeuedNode;
    }
    else
        return NULL;
}

void setupMap(){
    for (int row=0; row<maxRows; row++) {
        for (int col=0; col<maxCols; col++) {
            map[row][col].x=col;
            map[row][col].y=row;
            map[row][col].wayCost=FREE;
            map[row][col].totalCost=1;
        }
    }
    map[3][3].wayCost= START;
    map[7][7].wayCost= GOAL;
    
}

int distanceFromGoal(int x, int y){
    return  sqrt(pow(goalX-x,2)+pow(goalY-y,2));
}

void expandNode(struct Node * node){
    int x=node->x;
    int y=node->y;
    
    struct Node * nodeCandidate=&map[y-1][x];
    if (!nodeCandidate->closed) {
        nodeCandidate->totalCost=node->totalCost+nodeCandidate->wayCost+distanceFromGoal(nodeCandidate->x,nodeCandidate->y);
        nodeCandidate->parent=node;
        enqueueInSortedList(nodeCandidate);
    }
    nodeCandidate=&map[y+1][x];
    if (!nodeCandidate->closed) {
        nodeCandidate->totalCost=node->totalCost+nodeCandidate->wayCost+distanceFromGoal(nodeCandidate->x,nodeCandidate->y);
        nodeCandidate->parent=node;
        enqueueInSortedList(nodeCandidate);
    }
    nodeCandidate=&map[y][x-1];
    if (!nodeCandidate->closed) {
        nodeCandidate->totalCost=node->totalCost+nodeCandidate->wayCost+distanceFromGoal(nodeCandidate->x,nodeCandidate->y);
        nodeCandidate->parent=node;
        enqueueInSortedList(nodeCandidate);
    }
    nodeCandidate=&map[y][x+1];
    if (!nodeCandidate->closed) {
        nodeCandidate->totalCost=node->totalCost+nodeCandidate->wayCost+distanceFromGoal(nodeCandidate->x,nodeCandidate->y);
        nodeCandidate->parent=node;
        enqueueInSortedList(nodeCandidate);
    }
    node->closed=true;
}

void planRoute(struct Node * fromNode,struct Node * toNode){// returns a list of waypoints
    enqueueInSortedList(fromNode);    
    while (openList.node) {
        expandNode(dequeueNodeFromOpenList());
    }
}

int main(int argc, const char * argv[])
{
    setupMap();
    planRoute(&map[3][3],&map[7][7]);
    return 0;
}