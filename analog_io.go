package goduino

func (ino *Goduino) AnalogWrite(pin, value int) error {
	// XXX Below PinMode checking is not enabled because PWM mode also can use AnalogWrite
	// Check if pin is configured as analog
	//if ino.board.Pins()[p].Mode != Analog {
	//	if err = ino.PinMode(pin, Analog); err != nil {
	//		return
	//	}
	//}
	ino.logger.Printf("analogWrite(%d) -> %d\r\n", pin, value)
	return ino.board.AnalogWrite(pin, value)
}

// AnalogRead retrieves value from analog pin.
// Returns -1 if the response from the board has timed out
func (ino *Goduino) AnalogRead(pin int) (value int, err error) {
	p := ino.digitalPin(pin)
	// Check if pin is configured as analog
	if ino.board.Pins()[p].Mode != Analog {
		if err = ino.PinMode(pin, Analog); err != nil {
			return
		}
	}
	value = ino.board.Pins()[p].Value
	ino.logger.Printf("analogRead(%d) -> %d\r\n", pin, value)
	return
}
