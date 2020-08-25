/// rssim.cpp - Functions which perform the actual simulations
/// Marc Brooker, 30 May 2006
/// Edited by Yaaseen Martin, 25 August 2020

#include <cmath>
#include <limits>
#include <stdexcept>
#include "rssim.h"
#include "rsworld.h"
#include "rsradar.h"
#include "rsdebug.h"
#include "rsparameters.h"
#include "rsantenna.h"
#include "rstarget.h"
#include "rsresponse.h"
#include "rsnoise.h"

using namespace rs;

namespace {

    /// Results of solving the bistatic radar equation and friends
    struct REResults {
        rsFloat power;
        rsFloat delay;
        rsFloat doppler;
        rsFloat phase;
        rsFloat noise_temperature;
    };

    /// Class for range errors in RE calculations
    class RangeError {
    };
    
    // Function to calculate angles in a 3-D triangle
    void get_tri_angles(Vec3 P1, Vec3 P2, Vec3 P3, rsFloat* tri_angles)
    {
        Vec3 a = P3-P1;
        Vec3 b = P2-P1;
        rsFloat d = DotProduct(a,b);
        rsFloat theta1 = acos(d/(a.Length()*b.Length())); // Angle P3-to-P1-to-P2

        a = P1-P2;
        b = P3-P2;
        d = DotProduct(a,b);
        rsFloat theta2 = acos(d/(a.Length()*b.Length())); // Angle P1-to-P2-to-P3

        a = P1-P3;
        b = P2-P3;
        d = DotProduct(a,b);
        rsFloat theta3 = acos(d/(a.Length()*b.Length())); // Angle P2-to-P3-to-P1

        // Return the angles in the order of input points
        tri_angles[0] = theta1;
        tri_angles[1] = theta2;
        tri_angles[2] = theta3;
    }

    /// Solve the radar equation for a given set of parameters
    void SolveRE(const Transmitter *trans, const Receiver *recv, const Target *targ, rsFloat time, rsFloat length, RadarSignal *wave, REResults &results)
    {
        // Get the positions in space of the three objects
        Vec3 trpos = trans->GetPosition(time);  // Tx
        Vec3 repos = recv->GetPosition(time);   // Rx
        Vec3 tapos = targ->GetPosition(time);   // Target initial position
        Vec3 tapos_end = targ->GetPosition(time+length);  // Target next position
        SVec3 transvec = SVec3(tapos-trpos);
        SVec3 recvvec = SVec3(tapos-repos);

        // Calculate the distances
        rsFloat Rt = transvec.length;
        rsFloat Rr = recvvec.length;

        // From here on, transvec and recvvec need to be normalized
        transvec.length = 1;
        recvvec.length = 1;

        // Sanity-check Rt and Rr and throw an exception if they are too small
        if ((Rt <= std::numeric_limits<rsFloat>::epsilon()) || (Rr <= std::numeric_limits<rsFloat>::epsilon()))
            throw RangeError();

        // Step 1: Calculate the delay (in seconds) experienced by the pulse; see "Delay Equation" in doc/equations/equations.tex
        results.delay = (Rt+Rr)/rsParameters::c();

        // Get the RCS
        rsFloat RCS = targ->GetRCS(transvec, recvvec);

        // Get the wavelength
        rsFloat Wl = rsParameters::c()/wave->GetCarrier();

        //Get the system antenna gains (which include loss factors)
        rsFloat Gt = trans->GetGain(transvec, trans->GetRotation(time), Wl);
        rsFloat Gr = recv->GetGain(recvvec, recv->GetRotation(results.delay+time), Wl);

        // Step 2: Calculate the received power using the narrowband bistatic radar equation; see "Bistatic Narrowband radar equation" in doc/equations/equations.tex
        results.power = Gt*Gr*RCS/(4*M_PI);
        if (!recv->CheckFlag(Receiver::FLAG_NOPROPLOSS))
            results.power *= (Wl*Wl)/(pow(4*M_PI, 2)*Rt*Rt*Rr*Rr);

        // If the transmitter and/or receiver are multipath duals, multiply by the loss factor; none, one, or both of Tx/Rx can "observe" a reflection
        if (trans->IsMultipathDual())
            results.power *= trans->MultipathDualFactor(); // If the transmitter is a dual, account for one reflection
        if (recv->IsMultipathDual())
            results.power *= recv->MultipathDualFactor(); // If the receiver is a dual, account for one reflection

        // Step 3: Calculate phase shift; see "Phase Delay Equation" in doc/equations/equations.tex
        results.phase = -results.delay*2*M_PI*wave->GetCarrier();

        // Step 4: Calculate doppler shift; calculate positions at the end of the pulse
        // Get Beta angle
        rsFloat Bistatic_angles[3];
        get_tri_angles(trpos, tapos, repos, Bistatic_angles);  // Angles from Tx, to tapos, to Rx
        rsFloat Beta = Bistatic_angles[1]; // Beta angle in bistatic triangle [rad]

        // Get tapos_end projected onto the bistatic plane (i.e. next target coordinates)
        Vec3 normal = CrossProduct(tapos - trpos, tapos - repos);  // Returns a, b, c for plane
        normal = normal/normal.Length(); // Unit vector of plane normal
        Vec3 tapos_end_proj = tapos_end - normal*DotProduct((tapos_end - tapos), normal); // Project tapos_end onto the bistatic plane
        // (See: https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d)

        // Get delta using bistatic bisector and projected tapos_end
        Vec3 TxT0 = trpos - tapos;	// Shift Tx to tapos as the origin
        Vec3 RxT0 = repos - tapos;	// Shift Rx to tapos as the origin
        TxT0 = TxT0/TxT0.Length(); // Normalise to unit vector
        RxT0 = RxT0/RxT0.Length(); // Normalise to unit vector
        Vec3 Bisector = (TxT0 + RxT0) + tapos;  // Add normalised vectors (resultant is the bisecting vector), then shift result back by tapos
        rsFloat Delta_angles[3];
        get_tri_angles(Bisector, tapos, tapos_end_proj, Delta_angles); // To get delta
        rsFloat Delta = Delta_angles[1];  // Delta angle [rad]; if |Delta| > pi/2 rad, Fd will be negative (i.e. target moving away from radar)

        // Get Fd based on the velocity component projected onto the bistatic plane
        rsFloat Lambda = c/carrier; // Wavelength [m]
        rsFloat V_proj = (tapos_end_proj - tapos).Length()/length;  // Target velocity vector projected onto the bistatic plane
        rsFloat V_radial = V_proj*cos(Delta)*cos(Beta/2);	// Now project the V_proj onto the bistatic bisector, i.e. get radial velocity
        //results[i].doppler = 2*V_radial/Lambda; // Calculates Fd using the geometry
        results[i].doppler = carrier*(((1 + V_radial/c)/(1 - V_radial/c)) - 1);  // Calculates Fd using Doppler effect equation; Fd = Fr - Fc; technically more accurate?

        // Step 5: Calculate system noise temperature; only use the receive antenna noise temperature for now
        results.noise_temperature = recv->GetNoiseTemperature(recv->GetRotation(time+results.delay));
    }

    /// Perform the first stage of simulation calculations for the specified pulse and target
    void SimulateTarget(const Transmitter *trans, Receiver *recv, const Target *targ, const World *world, const TransmitterPulse *signal)
    {

        // Get the simulation start and end time
        rsFloat start_time = signal->time;
        rsFloat end_time = signal->time + signal->wave->GetLength();

        // Calculate the number of interpolation points we need to add
        rsFloat sample_time = 1.0 / rsParameters::cw_sample_rate();
        rsFloat point_count = std::ceil(signal->wave->GetLength() / sample_time);

        // Create the response
        Response *response = new Response(signal->wave, trans);
        try {
            // Loop through and add interpolation points
            for (int i = 0; i < point_count; i++) {
                rsFloat stime = i * sample_time + start_time; // Time of the start of the sample
                REResults results;
                SolveRE(trans, recv, targ, stime, sample_time, signal->wave, results);
                InterpPoint point(results.power, stime + results.delay, results.delay, results.doppler, results.phase, results.noise_temperature);
                response->AddInterpPoint(point);
            }

            // Add one more point at the end
            REResults results;
            SolveRE(trans, recv, targ, end_time, sample_time, signal->wave, results);
            InterpPoint point(results.power, end_time + results.delay, results.delay, results.doppler, results.phase, results.noise_temperature);
            response->AddInterpPoint(point);
        }
        catch (RangeError &re) {
        throw std::runtime_error("Receiver or Transmitter too close to Target for accurate simulation");
        }

        // Add the response to the receiver
        recv->AddResponse(response);
    }

    /// Solve the radar equation and friends (doppler, phase, delay) for direct transmission
    void SolveREDirect(const Transmitter *trans, const Receiver *recv, rsFloat time, rsFloat length, const RadarSignal *wave, REResults &results)
    {
        // Calculate the vectors to and from the transmitter
        Vec3 tpos = trans->GetPosition(time);
        Vec3 rpos = recv->GetPosition(time);
        SVec3 transvec = SVec3(tpos-rpos);
        SVec3 recvvec = SVec3(rpos-tpos);

        // Calculate the range
        rsFloat R = transvec.length;

        // Normalize transvec and recvvec for angle calculations
        transvec.length = 1;
        recvvec.length = 1;

        // If the two antennas are not in the same position, this can be calculated
        if (R > std::numeric_limits<rsFloat>::epsilon())
        {
            // Step 1: Calculate the delay
            results.delay = R/rsParameters::c();

            // Calculate the wavelength
            rsFloat Wl = rsParameters::c()/wave->GetCarrier();

            // Get the antenna gains
            rsFloat Gt = trans->GetGain(transvec, trans->GetRotation(time), Wl);
            rsFloat Gr = recv->GetGain(recvvec, recv->GetRotation(time+results.delay), Wl);

            // Step 2: Calculate the received power
            results.power = Gt*Gr*Wl*Wl/(4*M_PI);
            if (!recv->CheckFlag(Receiver::FLAG_NOPROPLOSS))
                results.power *= 1/(4*M_PI*pow(R, 2));

            // Step 3: Calculate the doppler shift (if one of the antennas is moving)
            Vec3 tpos_end = trans->GetPosition(time+length);
            Vec3 rpos_end = recv->GetPosition(time+length);
            Vec3 trpos_end = tpos_end - rpos_end;
            rsFloat R_end = trpos_end.Length();

            // Calculate the Doppler shift
            rsFloat vdoppler = (R_end-R)/length;
            results.doppler = (rsParameters::c()+vdoppler)/(rsParameters::c()-vdoppler);

            // Receiver duals do not receive any direct transmissions
            // However, real receivers can receive direct transmissions from a transmitter dual
            if (trans->IsMultipathDual())
                results.power *= trans->MultipathDualFactor();

            // Step 4: Calculate phase shift
            results.phase = fmod(results.delay*2*M_PI*wave->GetCarrier(), 2*M_PI);

            // Step 5: Calculate noise temperature
            results.noise_temperature = recv->GetNoiseTemperature(recv->GetRotation(time+results.delay));
        }
        else {
            // If the receiver and transmitter are too close, throw a range error
            throw RangeError();
        }
    }

    /// Model the pulse which is received directly by a receiver from a transmitter
    void AddDirect(const Transmitter *trans, Receiver *recv, const World *world, const TransmitterPulse *signal)
    {
        // If receiver and transmitter share the same antenna - there can't be a direct pulse
        if (trans->IsMonostatic() && (trans->GetAttached() == recv))
            return;

        // Get the simulation start and end time
        rsFloat start_time = signal->time;
        rsFloat end_time = signal->time+signal->wave->GetLength();

        // Calculate the number of interpolation points we need to add
        rsFloat sample_time = 1.0/rsParameters::cw_sample_rate();
        rsFloat point_count = std::ceil(signal->wave->GetLength()/sample_time);

        // Create the CW response
        Response *response = new Response(signal->wave, trans);
        try {
            // Loop through and add interpolation points
            for (int i = 0; i < point_count; i++) {
                rsFloat stime = i*sample_time+start_time;
                REResults results;
                SolveREDirect(trans, recv, stime, sample_time, signal->wave, results);
                InterpPoint point(results.power, results.delay+stime, results.delay, results.doppler, results.phase, results.noise_temperature);
                response->AddInterpPoint(point);
            }

            // Add one more point at the end
            REResults results;
            SolveREDirect(trans, recv, end_time, sample_time, signal->wave, results);
            InterpPoint point(results.power, results.delay+end_time, results.delay, results.doppler, results.phase, results.noise_temperature);
            response->AddInterpPoint(point);
        }
        catch (RangeError &re) {
            throw std::runtime_error("Receiver or Transmitter too close to Target for accurate simulation");
        }

        // Add the response to the receiver
        recv->AddResponse(response);
    }

}

/// Simulate a transmitter-receiver pair with a pulsed transmission
void rs::SimulatePair(const Transmitter *trans, Receiver *recv, const World *world)
{
    // Get the number of pulses
    int pulses = trans->GetPulseCount();
    std::vector<Target*>::const_iterator targ;

    // Build a pulse
    TransmitterPulse* pulse = new TransmitterPulse();

    // Loop through the pulses
    for (int i = 0; i < pulses; i++) {
        trans->GetPulse(pulse, i);
        for (targ = world->targets.begin(); targ != world->targets.end(); targ++) {
            SimulateTarget(trans, recv, *targ, world, pulse);
        }

        // Check if direct pulses are being considered for this receiver
        if (!recv->CheckFlag(Receiver::FLAG_NODIRECT)) {
            // Add the direct pulses
            AddDirect(trans, recv, world, pulse);
        }
    }
    delete pulse;
}
