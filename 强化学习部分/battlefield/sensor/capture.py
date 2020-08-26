from Box2D import b2RayCastCallback


class callback_capture(b2RayCastCallback):
    """
    This class captures the closest hit shape.
    """

    def __init__(self, **kwargs):
        b2RayCastCallback.__init__(self)
        self.userData = None

    # Called for each fixture found in the query. You control how the ray proceeds
    # by returning a float that indicates the fractional length of the ray. By returning
    # 0, you set the ray length to zero. By returning the current fraction, you proceed
    # to find the closest point. By returning 1, you continue with the original ray
    # clipping.
    def ReportFixture(self, fixture, point, normal, fraction):
        self.userData = fixture.userData
        self.point = fixture.body.position
        self.fraction = fraction
        return fraction
