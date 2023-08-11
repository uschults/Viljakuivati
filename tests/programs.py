    elif(msg.topic == "fill_container_1"):
        if(data == "true"):
            # check if another program is already running
            if(not program_running.is_set()):
                # check if level_buttons are connected
                # pin 40 is the first container
                if(level_buttons):
                    if(not (GPIO.input(8))):
                        #print("Programm: täida punker")
                        program_running.set()
                        fill_thread = Thread(target= fill_container, args= (program_running, ))
                        fill_thread.start()
                    else:
                        publish("teade","Punker juba täis")
                        publish("fill_container_in", "false")
                        #print("Punker 1 juba täis")
                else:
                    publish("teade","Ei ole tasemeandurit")
                    publish("fill_container_in", "false")
                    #print("Ei ole tasemeandureid")
            else:
                publish("teade","Teine program juba käib")
                publish("fill_container_in", "false")

    elif(msg.topic == "fill_container_2"):
        if(data == "true"):
            if(not program_running.is_set()):
                program_running.set()
                publish("mootor1_in", "true")
                publish("mootor2_in", "true")
            else:
                publish("teade","Teine program juba käib")
                publish("fill_container_2_in", "false")
        else:
            publish("mootor1_in", "false")
            publish("mootor2_in", "false")
            program_running.clear()