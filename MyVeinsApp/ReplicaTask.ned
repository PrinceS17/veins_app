//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;


// network description file for your Veins Application. Add parameters here

simple ReplicaTask extends BaseWaveApplLayer
{       
       	@class(ReplicaTask);
        int K = default(2);			// # replica task 
        int m = default(10);		// # F's interval
        double delay_limit = default(0.8);
        string appName = default("For Task　Offload!");
        @signal[sig](type = "simtime_t");
        @signal[sig_r](type = "double");
        @statistic[sig_delay](title = "sig delay"; source = "sig"; record = vector, mean;interpolationmode = none);		// record: can be either vector or scalar, the former better       
        @statistic[reliability](title = "reliability"; source = "sig_r"; record = vector, mean;interpolationmode = none);	
        @figure[last_delay](type = text; pos = 100,100; text = "last delay: N/A");
        
}
