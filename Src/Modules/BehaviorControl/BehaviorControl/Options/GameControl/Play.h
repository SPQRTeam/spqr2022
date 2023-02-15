option(Play)
{
    initial_state(play)
    {
        action
        {
            if (theRole.role == Role::RoleType::goalie)
                Goalie2020();
            else if (theRole.role == Role::RoleType::defender)
                DefenderAU();
            else if (theRole.role == Role::RoleType::supporter)
                SupporterAU();
            else if (theRole.role == Role::RoleType::jolly)
            {
                /*if (state_time < (40000.f + getExtraTime())) StaticJolly();
                    else*/
                JollyAU();
            }
            else if (theRole.role == Role::RoleType::striker)
                StrikerAU();


            //Search
            else if (theRole.role == Role::RoleType::searcher_1)
                Guard(1);
            else if (theRole.role == Role::RoleType::searcher_2)
                Guard(2);
            else if (theRole.role == Role::RoleType::searcher_4)
            {
                //if we saw the ball in a Delta time
                if (theTeamBallModel.position.x() > -(theFieldDimensions.xPosPenaltyStrikerStartPosition-200) && theTeamBallModel.timeWhenLastSeen < 800.f)
                    Guard(4);
                else
                    Searcher(4);
            }
            else if (theRole.role == Role::RoleType::searcher_3)
                Searcher(3);
        }
    }
}
