#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D interval = (end-start)/(double)num_nodes;
    
        masses.push_back(new Mass(start, node_mass, false));
        for (int i = 1; i<=num_nodes; i++){
            masses.push_back(new Mass(start+i*interval, node_mass, false));
            springs.push_back(new Spring(masses[i-1],masses[i],k));
        }

        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            //TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto a=s->m1->position, b = s->m2->position;
            auto fab = -s->k*((a-b)/(a-b).norm())*((a-b).norm()-s->rest_length);
            auto fba = -s->k*((b-a)/(b-a).norm())*((b-a).norm()-s->rest_length);
            s->m1->forces+=fab;
            s->m2->forces+=fba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                m->forces += m->mass * gravity;
                m->velocity = m->velocity + m->forces*delta_t;
                m->position = m->position+m->velocity*delta_t;

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            double len = (s->m1->position - s->m2->position).norm() - s->rest_length;
            len = len/2.0;
            auto dir = (s->m1->position - s->m2->position)/(s->m1->position - s->m2->position).norm();
            s->m2->forces += dir*len;
            s->m1->forces -=dir*len;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                m->forces += m->mass * gravity;
                m->position += 0.9995*m->forces*delta_t*delta_t;
                m->position += m->position-temp_position;

                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
