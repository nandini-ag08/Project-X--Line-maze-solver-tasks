//Pseudo code for PID control loop

setup:
    calibrate()                      // sweep sensors, record min/max per sensor

loop every 10 ms:                    // fixed dt
    for i in 0..4:
        cal[i] = rescale(raw[i], min[i], max[i])    // 0..1000, line = high

    total = sum(cal)
    if total == 0:                                   // line lost (all-zero case, checked before dividing)
        if lostFor() > TIMEOUT: stop motors; stay stopped
        else:  pivot toward side of lastPosition
        continue

    position   = sum(weight[i] * cal[i]) / total     // -2 .. +2
    lastPosition = position
    error      = position
    derivative = (error - lastError) / dt            // low-pass filtered
    correction = Kp*error + Kd*derivative            // Ki = 0
    lastError  = error

    left  = clamp(base + correction + leftTrim, -255, 255)
    right = clamp(base - correction,            -255, 255)
    drive(left, right)



