Main function Flow

- Press key 'c'
    - TopView
    - getMainAngles
    - getMainDirections
    - getDistanceToSeePlane (min, max)
- alignQuadcopterToCurrentPlane()
- Wait for 4 points to be clicked
- Press key 'X'
- captureTheCurrentPlane()
- adjustForNextCapture()
- alignQuadcopterToNextPlane()

alignQuadcopterToCurrentPlane() -> adjustYawToCurrentPlane()
                                -> adjustTopBottomEdges()
                                -> adjustLeftEdge()

adjustYawToCurrentPlane()   -> doJLinkage()
                            -> findAngles()
                            -> moveDroneViaSetOfPoints()

adjustTopBottomEdges()  -> getPointToPlaneDistance()
                        -> designPathForDroneRelative()
                        -> getHeightFromGround()

adjustLeftEdge() -> checkVisibility()
                 -> designPathForDroneRelative()

doJLinkage()    -> getMultiplePlanes3d()
                -> getCurrentPlaneIndex()
                -> getCompleteCurrentPlaneInfo()

getMultiplePlanes3d()   -> findPercBoundEachPlane()
                        -> orderPlanesFromQuadcopterPosition()
                        -> clear2dVector()

getCompleteCurrentPlaneInfo() -> checkPlaneParametersSign()

captureTheCurrentPlane() -> setRender()
                         -> extractBoundingPoly()
                         -> setContinuousBoundingBoxPoints()
                         -> setSigPlaneBoundingBoxPoints()
                         -> setVisitedBoundingBoxPoints()
                         -> renderFrame()
                         -> getDistanceToSeePlane()
                         -> moveForward()/moveBackward()/moveUp()/moveDown()
                         -> isNewPlaneVisible()
                         -> checkVisibility()
                         -> alignQuadcopterToNextPlane()
                         -> sendLand()
                         -> augmentInfo()
                         -> adjustForNextCapture()
                         -> WriteInfoToFile()

adjustForNextCapture() -> -> rotateClockwise()
                          -> moveBackward()/moveForward()
                          -> doJLinkage()
                          -> isNewPlaneVisible()
                          -> getPointToPlaneDistance()
                          -> checkVisibility()
                          -> copyNecessaryInfo()
                          -> rotateCounterClockwise()
                          -> moveRight()
                       -> -> moveRight()
                          -> designPathToChangeYaw()
                          -> checkVisiblity()
                          -> copyNecessaryInfo()
                       -> WriteInfoToFile()
                       -> alignQuadcopterToNextPlane()



alignQuadcopterToNextPlane() -> -> moveRight()
                                -> doJLinkage()
                                -> setContinuousBoundingBoxPoints()
                                -> setSigPlaneBoundingBoxPoints()
                                -> setRender()
                                -> renderFrame()
                                -> rotateCounterClockwise()
                                -> moveInDirection()
                                -> moveDown()/moveUp()
                                -> convertWRTQuadcopterOrigin()
                                -> alignQuadcopterToCurrentPlane()
                                -> adjustLeftEdge()
                             -> -> moveBackward()
                                -> moveForward()
                                -> rotateClockwise()
                                -> doJlinkage()
                                -> moveBackward()/moveForward()
                                -> alignQuadcopterToCurrentPlane()
                                -> adjustLeftEdge()
