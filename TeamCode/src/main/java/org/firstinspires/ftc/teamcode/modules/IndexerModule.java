package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class IndexerModule {
    public IndexerSpot artifact_1;
    public IndexerSpot artifact_2;
    public IndexerSpot artifact_3;

    public List<IndexerSpot> artifacts;
    public int num_artifacts = 0;

    public enum ModuleStates {
        WAITING,
        SHOOT_INDIVIDUAL,
        SHOOT_ALL;
    }
    public ModuleStates module_state = ModuleStates.WAITING;

    private IndexerSpot last_fired_indexer;

    public IndexerModule(Servo a1_servo, RevColorSensorV3 a1_color_a, RevColorSensorV3 a1_color_b,
                         Servo a2_servo, RevColorSensorV3 a2_color_a, RevColorSensorV3 a2_color_b,
                         Servo a3_servo, RevColorSensorV3 a3_color_a, RevColorSensorV3 a3_color_b)
    {
        this.artifact_1 = new IndexerSpot(a1_servo, a1_color_a, a1_color_b);
        this.artifact_2 = new IndexerSpot(a2_servo, a2_color_a, a2_color_b);
        this.artifact_3 = new IndexerSpot(a3_servo, a3_color_a, a3_color_b);

        this.artifacts.add(this.artifact_1);
        this.artifacts.add(this.artifact_2);
        this.artifacts.add(this.artifact_3);
    }

    public void update()
    {
        this.artifact_1.update();
        this.artifact_2.update();
        this.artifact_3.update();

        /* Recount the number of artifacts currently in the indexer */
        this.num_artifacts = 0;
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status != ArtifactStatus.EMPTY)
            {
                this.num_artifacts++;
            }
        }

        switch (this.module_state)
        {
            case SHOOT_INDIVIDUAL:
                /* If for whatever reason we no longer have a last fired indexer, or the last
                /* fired indexer is just returned to home, return to the waiting state. */
                if (this.last_fired_indexer == null || this.last_fired_indexer.isHome())
                {
                    this.module_state = ModuleStates.WAITING;
                }

                break;
            case SHOOT_ALL:
                /* If there is a spot we just shot and it has returned back home, shoot the next spot */
                if (this.last_fired_indexer == null || (this.last_fired_indexer != null && this.last_fired_indexer.isHome()))
                {
                    boolean shot_next = this.shootNext();

                    /* If we ran out of artifacts to shoot, return to waiting state */
                    if (!shot_next)
                    {
                        this.module_state = ModuleStates.WAITING;
                    }
                }

                break;
        }
    }

    public void shootGreen()
    {
        /* Loop through our artifacts, if one of them is green shoot it and exit this method. */
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status == ArtifactStatus.GREEN)
            {
                spot.shoot();
                this.last_fired_indexer = spot;
                return;
            }
        }

        /* We looped through our artifacts and we didn't find a green...
        /* Loop through them again and just shoot the first one that isn't empty */
        this.shootNext();
    }

    public void shootPurple()
    {
        /* Loop through our artifacts, if one of them is purple shoot it and exit this method. */
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status == ArtifactStatus.PURPLE)
            {
                spot.shoot();
                this.last_fired_indexer = spot;
                return;
            }
        }

        /* We looped through our artifacts and we didn't find a green...
        /* Loop through them again and just shoot the first one that isn't empty */
        this.shootNext();
    }

    public boolean shootNext()
    {
        for (IndexerSpot spot : this.artifacts)
        {
            if (spot.artifact_status != ArtifactStatus.EMPTY)
            {
                spot.shoot();
                this.last_fired_indexer = spot;
                /* True indicates that an artifact was found to shoot */
                return true;
            }
        }

        /* False indicates that there are no more artifacts to shoot */
        this.last_fired_indexer = null;
        return false;
    }

    public void shootAll()
    {
        this.module_state = ModuleStates.SHOOT_ALL;
    }

}
