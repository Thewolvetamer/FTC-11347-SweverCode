package org.firstinspires.ftc.teamcode;

public class ButtonRebounce {
    /**
     * Holds all the states that a toggle can be in. When pressing a button, there are 3 states:
     * 1. Not begun
     * 2. In progress
     * 3. Complete
     *
     * If you're checking a button press using a conventional on/off check and using it to
     * flip a boolean, then you'll flip once for every time the button is held and the
     * loop iterates.
     */
    enum Status
    {
        NOT_BEGUN ,
        IN_PROGRESS ,
        COMPLETE
    }


    private Status _status = Status.NOT_BEGUN;      // Current status of the toggle


    /**
     *  Monitors and adjusts the toggle value based on previous toggle values and the
     *  state of the boolean passed in.
     */
    //0 is not begun 1 in progress 2 is complete
    public void reset_status(){
        _status=Status.NOT_BEGUN;

    }
    final public Status status(boolean buttonStatus)
    {
        // If the button is being held
        if(buttonStatus && _status == Status.NOT_BEGUN)
            _status = Status.IN_PROGRESS;

            // If the button is not being pressed and the toggle was in progress
        else if(!buttonStatus && _status == Status.IN_PROGRESS)
            _status = Status.COMPLETE;

            // If the toggle is finished
        else if(_status == Status.COMPLETE)
            _status = Status.NOT_BEGUN;

        return _status;
    }
}
//Implementation:
//
//        ButtonRebounce slowToggle = new ButtonRebounce();
//
//        if(slowToggle.status(gamepad1.a) == ButtonRebounce.Status.COMPLETE)
//        power.toggleSlow();

