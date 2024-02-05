This Router project has been designed on 3-layer protocol. This protocol consists 2-phases :-
                                                                                             1. Encapsulation.
                                                                                             2. De-encapsulation.
In "encapsulation" in 1st layer TCP header is added to the data, in 2nd layer IP address id addded and in 3rd layer MAC address id added to data. In "de-encapsulation" the reverse of encapsulation is done.
Each address consists of both adresses of Transmitter and riceiver.
In this router project it is assumed that the decoding of data address has been done and received by router.
Data to this router is sent i  bytes form.
First byte of every data packet is "Header byte" in which 1st 2 LSB bits are address of destinantion device (FIFO) and remaining 6 bits indicate number of payload data to be received.
Last byte of every data packet consists of "parity byte".
Input signals of top block :-
                            1. clocl, reset signal, data_in
                            2. pkt_valid - remains high for header bute and data payloads.
                            3. read_end - for read pin of FIFO.
Output signals of Top block :-
                            1. data_out
                            2. valid_out - becomes "high" if received data is readable.
                            3. error - if calculated parity doesn't match recieved parity.
                            4. busy - if FIFO is full then busy signal becomes high to halt the data transmission.
                            
