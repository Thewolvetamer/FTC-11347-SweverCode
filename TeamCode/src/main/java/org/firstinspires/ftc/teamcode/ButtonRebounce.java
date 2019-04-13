package org.firstinspires.ftc.teamcode;

public class ButtonRebounce {
    /**
     * Holds all the states that a toggle can be in. When pressing a button, there are 3 states:
     * 1. Not begun
     * 2. In progress
     * 3. Complete
     *
     *
     * If you're checking a button press using a conventional on/off check and using it to
     * flip a boolean, then you'll flip once for every time the button is held and the
     * loop iterates.
     */
    enum Status
    {
        UNPRESSED ,
        PRESSED,
        RELEASED
    }


    private Status _status = Status.UNPRESSED;      // Current status of the toggle


    /**
     *  Monitors and adjusts the toggle value based on previous toggle values and the
     *  state of the boolean passed in.
     */
    final public Status status(boolean buttonStatus)
    {
        // If the button is being held
        if(buttonStatus && _status == Status.UNPRESSED)
            _status = Status.PRESSED;

            // If the button is not being pressed and the toggle was in progress
        else if(!buttonStatus && _status == Status.PRESSED)
            _status = Status.RELEASED;

            // If the toggle is finished
        else if(_status == Status.RELEASED)
            _status = Status.UNPRESSED;

        return _status;
    }
}
//Implementation:
//
//        ButtonRebounce slowToggle = new ButtonRebounce();
//
//        if(slowToggle.status(gamepad1.a) == ButtonRebounce.Status.RELEASED)
//        power.toggleSlow();

