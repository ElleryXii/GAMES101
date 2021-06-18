#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D cur = Vector2D(start.x, start.y);
        Vector2D interval = (end-start)/(double)num_nodes;
        while (cur.x <= end.x && cur.y<=end.y){
            masses.push_back(&Mass(cur, node_mass, false));
            cur += interval;
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto ab =s->m2->position - s->m1->position;
            auto f = s->k*(ab/ab.norm())*(ab.norm()-s->rest_length);
            s->m1->forces+=f;
            s->m2->forces-=f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                
                // TODO (Part 2): Add global damping
                float k_d_global = 0.01;
                m->forces += - k_d_global * m->velocity;
                

                Vector2D a = m->forces;

                //semi-implicit Euler
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;

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
            auto ab =s->m2->position - s->m1->position;
            auto f = s->k*(ab/ab.norm())*(ab.norm()-s->rest_length);
            s->m1->forces+=f;
            s->m2->forces-=f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                auto a = m->forces / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                auto temp_location = m->position;
                // TODO (Part 4): Add global Verlet damping
                float dampfactor = 0.00005;
                m->position = m->position +  (1.0 - dampfactor) * (m->position - m->last_position) + a * delta_t *delta_t;
                m->last_position = temp_location;
            }
            m->forces =  Vector2D(0,0);
        }
    }
}
